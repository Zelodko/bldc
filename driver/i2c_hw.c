#include "i2c_hw.h"
#include "string.h"

/* Default timeout in ticks for hardware transfers */
#ifndef I2C_HW_TIMEOUT_TICKS
#define I2C_HW_TIMEOUT_TICKS 1000
#endif

/* Helper to pick a default driver if none attached.
 * If user defines HW_I2C_DEV in hwconfig, use it; else return NULL.
 */
static I2CDriver *pick_default_driver(void) {
#ifdef HW_I2C_DEV
	return &HW_I2C_DEV;
#else
	return NULL;
#endif
}

/* Basic initialization - mutex and clear state.
 * If driver is already set in state, that driver is used; otherwise
 * the driver will be picked lazily from pick_default_driver().
 */
void i2c_bb_init(i2c_bb_state *s) {
	chMtxObjectInit(&s->mutex);
	s->has_started = false;
	s->has_error = false;
	s->transaction_open = false;
	s->txlen = 0;

	/* if no driver chosen yet, deferred to pick_default_driver in calls */
	if (s->i2c == NULL) {
		s->i2c = pick_default_driver();
	}
}

/* Handy initializer that binds a specific ChibiOS I2CDriver */
void i2c_bb_init_with_driver(i2c_bb_state *s, I2CDriver *driver) {
	s->i2c = driver;
	i2c_bb_init(s);
}

void i2c_bb_set_driver(i2c_bb_state *s, I2CDriver *driver) {
    /* Mutex may not be initialized yet */
    if (s->mutex.owner == NULL) {
        s->i2c = driver;
        return;
    }

    chMtxLock(&s->mutex);
    s->i2c = driver;
    chMtxUnlock(&s->mutex);
}

I2CDriver *i2c_bb_get_driver(i2c_bb_state *s) {
	I2CDriver *drv;
	chMtxLock(&s->mutex);
	drv = s->i2c ? s->i2c : pick_default_driver();
	chMtxUnlock(&s->mutex);
	return drv;
}

/* Attempt to restore the bus. For HW driver we acquire/release bus and
 * attempt a small transfer that may clear minor bus errors. If real bus
 * unsticking (clock toggles) is required, do it elsewhere (GPIO toggle).
 */
void i2c_bb_restore_bus(i2c_bb_state *s) {
	chMtxLock(&s->mutex);

	I2CDriver *i2c = s->i2c ? s->i2c : pick_default_driver();
	if (i2c == NULL) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return;
	}

	i2cAcquireBus(i2c);
	/* zero-length transmit to address 0 may clear peripheral state (best-effort) */
	(void)i2cMasterTransmitTimeout(i2c, 0, NULL, 0, NULL, 0, I2C_HW_TIMEOUT_TICKS);
	(void)i2cGetErrors(i2c);
	i2cReleaseBus(i2c);

	s->has_started = false;
	s->has_error = false;
	s->transaction_open = false;
	s->txlen = 0;

	chMtxUnlock(&s->mutex);
}

/* Core transactional API: single call that does start, tx, rx, stop */
bool i2c_bb_tx_rx(i2c_bb_state *s, uint16_t addr,
                  uint8_t *txbuf, size_t txbytes,
                  uint8_t *rxbuf, size_t rxbytes)
{
	chMtxLock(&s->mutex);
	s->has_error = false;

	I2CDriver *i2c = s->i2c ? s->i2c : pick_default_driver();
	if (i2c == NULL) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return false;
	}

	i2cAcquireBus(i2c);
	msg_t r = i2cMasterTransmitTimeout(i2c, (i2caddr_t)addr, txbuf, txbytes, rxbuf, rxbytes, I2C_HW_TIMEOUT_TICKS);
	i2cflags_t errs = i2cGetErrors(i2c);
	i2cReleaseBus(i2c);

	if (r != MSG_OK || errs != 0) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return false;
	}

	s->has_started = false;
	s->transaction_open = false;
	s->txlen = 0;
	chMtxUnlock(&s->mutex);
	return true;
}

/* --- Minimal buffered emulation for legacy per-byte API --- */

/*
 * i2c_bb_write_byte:
 * - If send_start is true: caller is sending the address byte (addr<<1 | rw).
 *   We parse and store the 7-bit addr and rw flag and perform a small probe
 *   (0-length write) for write transactions to detect NACK early.
 * - If send_start is false: append 'byte' to tx buffer.
 * - If send_stop is true: flush (send) the buffered tx bytes to stored addr.
 *
 * Returns: true if NACK or error (matches older function which returned 'nack')
 */
bool i2c_bb_write_byte(i2c_bb_state *s, bool send_start, bool send_stop, unsigned char byte) {
	chMtxLock(&s->mutex);
	s->has_error = false;

	I2CDriver *i2c = s->i2c ? s->i2c : pick_default_driver();
	if (i2c == NULL) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return true; /* treat as NACK/error */
	}

	/* If caller issues a START with an address byte (addr<<1 | rw) */
	if (send_start) {
		uint8_t addr7 = (uint8_t)(byte >> 1);
		uint8_t rw = (byte & 0x01) ? 1 : 0;
		s->addr = addr7;
		s->rw = (rw != 0);
		s->transaction_open = true;
		s->txlen = 0;
		s->has_started = true;

		/* For write-direction, probe the address (zero-length write) to detect NACK */
		if (s->rw == 0) {
			i2cAcquireBus(i2c);
			msg_t r = i2cMasterTransmitTimeout(i2c, (i2caddr_t)s->addr, NULL, 0, NULL, 0, I2C_HW_TIMEOUT_TICKS);
			i2cflags_t errs = i2cGetErrors(i2c);
			i2cReleaseBus(i2c);
			if (r != MSG_OK || errs != 0) {
				s->has_error = true;
				chMtxUnlock(&s->mutex);
				return true; /* NACK/error */
			}
			/* ACK */
			chMtxUnlock(&s->mutex);
			return false;
		} else {
			/* read-mode: no immediate check here, read happens in read_byte */
			chMtxUnlock(&s->mutex);
			return false; /* assume ACK for start */
		}
	}

	/* Not send_start: treat 'byte' as payload to transmit */
	if (!send_start) {
		/* in read-mode writing payload is unusual, but allow append */
		if (s->txlen < sizeof(s->txbuf)) {
			s->txbuf[s->txlen++] = (uint8_t)byte;
		} else {
			s->has_error = true;
			chMtxUnlock(&s->mutex);
			return true;
		}
	}

	/* If caller asked for stop, flush the buffered tx */
	if (send_stop) {
		if (!s->transaction_open) {
			/* Nothing to send; treat as success */
			s->transaction_open = false;
			s->txlen = 0;
			s->has_started = false;
			chMtxUnlock(&s->mutex);
			return false;
		}

		/* For write-mode, transmit the buffered data to stored address */
		if (!s->rw) {
			i2cAcquireBus(i2c);
			msg_t r = MSG_OK;
			if (s->txlen > 0) {
				r = i2cMasterTransmitTimeout(i2c, (i2caddr_t)s->addr, s->txbuf, s->txlen, NULL, 0, I2C_HW_TIMEOUT_TICKS);
			} else {
				/* zero-length write (already probed earlier), treat as success */
				r = MSG_OK;
			}
			i2cflags_t errs = i2cGetErrors(i2c);
			i2cReleaseBus(i2c);

			if (r != MSG_OK || errs != 0) {
				s->has_error = true;
				s->transaction_open = false;
				s->txlen = 0;
				s->has_started = false;
				chMtxUnlock(&s->mutex);
				return true; /* NACK/error */
			}
		} else {
			/* read-mode + send_stop: nothing to flush here (reads are done in read_byte) */
		}

		/* reset buffered state */
		s->transaction_open = false;
		s->txlen = 0;
		s->has_started = false;
		chMtxUnlock(&s->mutex);
		return false; /* ACK/no error */
	}

	chMtxUnlock(&s->mutex);
	return false;
}

/*
 * i2c_bb_read_byte:
 * - If transaction was opened with a read address (send_start with LSB=1),
 *   each read_byte call will perform a single-byte read via the HW driver.
 * - nack parameter is informational (ignored for HW), send_stop closes the
 *   transaction if requested.
 *
 * Returns the read byte (0 on error, and s->has_error set).
 */
unsigned char i2c_bb_read_byte(i2c_bb_state *s, bool nack, bool send_stop) {
	(void)nack;
	chMtxLock(&s->mutex);
	s->has_error = false;

	I2CDriver *i2c = s->i2c ? s->i2c : pick_default_driver();
	if (i2c == NULL) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return 0;
	}

	uint8_t ret = 0;

	if (!s->transaction_open || !s->rw) {
		/* No read-mode open; return error */
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return 0;
	}

	i2cAcquireBus(i2c);
	msg_t r = i2cMasterTransmitTimeout(i2c, (i2caddr_t)s->addr, NULL, 0, &ret, 1, I2C_HW_TIMEOUT_TICKS);
	i2cflags_t errs = i2cGetErrors(i2c);
	i2cReleaseBus(i2c);

	if (r != MSG_OK || errs != 0) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return 0;
	}

	if (send_stop) {
		s->transaction_open = false;
		s->has_started = false;
	}

	chMtxUnlock(&s->mutex);
	return ret;
}
