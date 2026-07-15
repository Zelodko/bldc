/*
	Copyright 2026 Lukas Hrazky

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "transport_spi_hw.h"
#include <string.h>

// SPI read sets bit 7 of the register address, write clears it.
#define SPI_READ_BIT 0x80

// H7's SPIv3 splits config across CFG1 (baud rate + data size) and CFG2 (mode). MBR is a
// 3-bit prescaler field (SPI clock = kernel clock / 2^(MBR+1)), and DSIZE encodes bits-per-frame
// minus one - using the raw bit count here (8) would configure 9-bit frames and desync every
// byte boundary on the wire (see imu.c's SPI_DATASIZE_8BIT for the same fix, done first there).
#define SPI_BaudRatePrescaler_MBR(n)	((n) << SPI_CFG1_MBR_Pos)
#define SPI_DATASIZE_8BIT				7
#define SPI_MODE_0						0
#define SPI_MODE_1						SPI_CFG2_CPHA
#define SPI_MODE_2						SPI_CFG2_CPOL
#define SPI_MODE_3						(SPI_CFG2_CPOL | SPI_CFG2_CPHA)

// Default IMU SPI clock used when bus_hz is 0.
#define SPI_HW_DEFAULT_HZ	10500000

static SPIDriver *dev_of(transport_t *t) {
	return t->bus.spi_hw.spid;
}

// Installed as cfg.error_cb, which the H7 SPI LLD does invoke on a DMA/framing error
// (__spi_isr_error_code() in hal_spi_v2.h calls spip->config->error_cb). The driver-level
// app_arg/err_cb pair that ChibiOS_21_11_3's SPI_DRIVER_EXT_FIELDS (halconf.h) adds is a
// separate, purely app-defined mechanism (used by the encoder SPI drivers to pass per-transfer
// context into their data_cb) that nothing in the LLD calls automatically - only app_arg is
// reused here, as a place to stash this call's error flag.
static void spi_err_cb(SPIDriver *spip) {
	if (spip->app_arg) {
		*(volatile bool *)spip->app_arg = true;
	}
}

static bool read_reg(transport_t *t, uint8_t dev_addr, uint8_t reg, uint8_t *rx, size_t len) {
	(void)dev_addr;
	if (len > IMU_MAX_BURST) {
		return false;
	}
	SPIDriver *spid = dev_of(t);
	uint8_t *txd = t->bus.spi_hw.txd;
	uint8_t *rxd = t->bus.spi_hw.rxd;

	txd[0] = reg | SPI_READ_BIT;
	memset(txd + 1, 0, len);

	volatile bool err = false;
	spiAcquireBus(spid);
	spid->app_arg = (void *)&err;
	spiSelect(spid);
	// One full-duplex exchange (address byte + len bytes read back), not a separate
	// send-then-receive: a single bus acquisition and DMA setup per read instead of
	// two. The halved per-read overhead matters at the sampling burst rate.
	spiExchange(spid, 1 + len, txd, rxd);
	spiUnselect(spid);
	spid->app_arg = NULL;
	spiReleaseBus(spid);

	if (err) {
		return false;
	}
	memcpy(rx, rxd + 1, len);
	return true;
}

static bool write_reg(transport_t *t, uint8_t dev_addr, uint8_t reg, const uint8_t *tx, size_t len) {
	(void)dev_addr;
	if (len > IMU_MAX_BURST) {
		return false;
	}
	SPIDriver *spid = dev_of(t);
	uint8_t *txd = t->bus.spi_hw.txd;
	uint8_t *rxd = t->bus.spi_hw.rxd;

	txd[0] = reg & ~SPI_READ_BIT;
	memcpy(txd + 1, tx, len);

	volatile bool err = false;
	spiAcquireBus(spid);
	spid->app_arg = (void *)&err;
	spiSelect(spid);
	// spiExchange(), not spiSend(): a TX-only DMA transfer still has to put the inevitable
	// full-duplex RX data somewhere, and spiSend() routes it to the SPIDriver's own internal
	// rxsink field. That field lives in SPID1 itself, a plain global - on this port plain
	// globals default to DTCM (see the .nocache comment on m_transport in imu.c), which no DMA
	// controller can reach, so every spiSend() transfer error'd out (confirmed on hardware).
	// Exchanging into our own .nocache rxd buffer and discarding it sidesteps that entirely.
	spiExchange(spid, 1 + len, txd, rxd);
	spiUnselect(spid);
	spid->app_arg = NULL;
	spiReleaseBus(spid);

	return !err;
}

static uint16_t max_sample_rate(transport_t *t) {
	(void)t;
	return 10000;
}

static const transport_interface_t spi_hw_interface = {
	.name = "spi-hw",
	.max_sample_rate = max_sample_rate,
	.read_reg = read_reg,
	.write_reg = write_reg,
	.recover = NULL,
	.deinit = NULL,
};

// CFG1.MBR field value (0-7) for the fastest prescaler whose SPI clock does not
// exceed bus_hz (0 = SPI_HW_DEFAULT_HZ). SPI clock = kernel_clk / 2^(mbr + 1).
static uint8_t hz_to_mbr(SPIDriver *spid, uint32_t bus_hz) {
	// Unlike F4 (SPI1 off PCLK2, SPI2/3 off PCLK1), H7's SPI1/2/3 kernel clock is a
	// separate mux (STM32_SPI123SEL, PLL1_Q by default here) shared by all three, and
	// SPI4/5/6 have their own selectors again - so each enabled instance needs its own
	// STM32_SPIxCLK, there's no single PCLKn shortcut.
	uint32_t kclk;
#if STM32_SPI_USE_SPI1
	if (spid == &SPID1) {
		kclk = STM32_SPI1CLK;
	} else
#endif
#if STM32_SPI_USE_SPI2
	if (spid == &SPID2) {
		kclk = STM32_SPI2CLK;
	} else
#endif
#if STM32_SPI_USE_SPI3
	if (spid == &SPID3) {
		kclk = STM32_SPI3CLK;
	} else
#endif
#if STM32_SPI_USE_SPI4
	if (spid == &SPID4) {
		kclk = STM32_SPI4CLK;
	} else
#endif
#if STM32_SPI_USE_SPI5
	if (spid == &SPID5) {
		kclk = STM32_SPI5CLK;
	} else
#endif
#if STM32_SPI_USE_SPI6
	if (spid == &SPID6) {
		kclk = STM32_SPI6CLK;
	} else
#endif
	{
		kclk = STM32_SPI1CLK;
	}

	if (bus_hz == 0) {
		bus_hz = SPI_HW_DEFAULT_HZ;
	}

	for (uint8_t mbr = 0; mbr < 7; mbr++) {
		if ((kclk >> (mbr + 1)) <= bus_hz) {
			return mbr;
		}
	}

	return 7; // equals prescaler /256
}

void transport_spi_hw_init(transport_t *t, SPIDriver *spid, uint32_t af,
		stm32_gpio_t *nss_gpio, uint8_t nss_pin, stm32_gpio_t *sck_gpio, uint8_t sck_pin,
		stm32_gpio_t *mosi_gpio, uint8_t mosi_pin, stm32_gpio_t *miso_gpio, uint8_t miso_pin,
		uint32_t bus_hz) {
	t->interface = &spi_hw_interface;
	t->bus.spi_hw.spid = spid;

	palSetPadMode(nss_gpio, nss_pin,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(sck_gpio, sck_pin,
			PAL_MODE_ALTERNATE(af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(mosi_gpio, mosi_pin,
			PAL_MODE_ALTERNATE(af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(miso_gpio, miso_pin,
			PAL_MODE_ALTERNATE(af) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_FLOATING);

	t->bus.spi_hw.cfg = (SPIConfig){
		false, // circular
		false, // slave
		NULL, // data callback
		spi_err_cb, // error callback
		nss_gpio, // ssport
		nss_pin, // sspad
		// cfg1 is uint32_t (unlike F4's single 16-bit CR1) - a (uint16_t) cast here would
		// truncate MBR clean off (SPI_CFG1_MBR_Pos is 28), silently forcing the fastest
		// prescaler regardless of bus_hz and clocking the slave far outside its rated speed.
		SPI_BaudRatePrescaler_MBR(hz_to_mbr(spid, bus_hz)) | SPI_DATASIZE_8BIT, // cfg1
		SPI_MODE_3, // cfg2
	};
	spiStart(spid, &t->bus.spi_hw.cfg);
}
