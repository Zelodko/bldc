/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "utils_math.h"
#include "mc_interface.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "utils.h"
#include "app.h"
#include "mempools.h"
#include "timeout.h"

#include <math.h>
#include <stdio.h>

// Variables
static volatile bool i2c_running = false;

// I2C configuration

static const I2CConfig i2cfg = {
  .timingr          = STM32_TIMINGR_PRESC(15U) | STM32_TIMINGR_SCLDEL(4U) |
                      STM32_TIMINGR_SDADEL(2U) | STM32_TIMINGR_SCLH(15U) |
                      STM32_TIMINGR_SCLL(21U),
  .cr1              = 0,
  .cr2              = 0
};

static void terminal_button_test(int argc, const char **argv);
static void terminal_shutdown_now(int argc, const char **argv);

static void terminal_cmd_doublepulse(int argc, const char** argv)
{
	(void)argc;
	(void)argv;

    int preface, pulse1, breaktime, pulse2;
    int utick;
    int deadtime = -1;

    if (argc < 5) {
        commands_printf("Usage: double_pulse <preface> <pulse1> <break> <pulse2> [deadtime]");
        commands_printf("   preface: idle time in us");
        commands_printf("    pulse1: high time of pulse 1 in us");
        commands_printf("      break: break between pulses in us");
        commands_printf("    pulse2: high time of pulse 2 in us");
        commands_printf("   deadtime: overwrite deadtime, in ns");
        return;
    }
    sscanf(argv[1], "%d", &preface);
    sscanf(argv[2], "%d", &pulse1);
    sscanf(argv[3], "%d", &breaktime);
    sscanf(argv[4], "%d", &pulse2);
    if (argc == 6) {
        sscanf(argv[5], "%d", &deadtime);
    }
    timeout_configure_IWDT_slowest();
	palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palClearPad(GPIOA, 10);
	mcpwm_deinit();
	mcpwm_foc_deinit();
	utick = (uint32_t)(SYSTEM_TIMER_CLOCK / 1000000U);


	rccResetTIM4();
	rccEnableTIM4(TRUE);

	// Configure TIM4 base
	TIM4->PSC = 0; // Prescaler
	TIM4->ARR = SYSTEM_TIMER_CLOCK / 20000; // Period (0-based)
	TIM4->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Counter Mode Up (default)
	TIM4->CR1 &= ~TIM_CR1_CKD; // Clock division = 1 (no division)
	// Generate update event to load prescaler immediately
	TIM4->EGR = TIM_EGR_UG;

	TIM4->SMCR |= TIM_SMCR_MSM; // Enable Master/Slave mode — set MSM bit in SMCR
	TIM4->CR2 &= ~TIM_CR2_MMS; // Select Trigger Output (TRGO) source: enable = counter enable as TRGO
	TIM4->CR2 |= TIM_CR2_MMS_0;           // Set MMS to 0x1 (Counter enable as TRGO)
	TIM4->CNT = 0; // Reset TIM4 counter

	rccResetTIM1();
	rccEnableTIM1(TRUE);
	// Time Base configuration
	TIM1->PSC = 0;
	TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	TIM1->CR1 &= ~TIM_CR1_CKD;
	TIM1->ARR = (preface + pulse1) * utick;
	TIM1->RCR = 0;
	TIM1->EGR = TIM_EGR_UG;


	// Channel 1, 2 and 3 Configuration

	// Set CH1 PWM Mode 2 (OC1M = 0b111)
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	// Set CH2 Inactive mode (OC2M = 0b100)
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_2;

	// Set CH3 Inactive mode (OC3M = 0b100)
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2;

	// Preload
	TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

	// 3. Set CCR values once
	TIM1->CCR1 = preface * utick;
	TIM1->CCR2 = preface * utick;
	TIM1->CCR3 = preface * utick;

	// 4. Clear polarity bits once (active high polarity default)
	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
	TIM1->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);

	// 5. Enable outputs once
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

	// 6. Generate COM event for immediate loading
	TIM1->EGR = TIM_EGR_COMG;

	// Calculate deadtime count (0-255) from your function
	uint8_t deadtime_val;
	if (deadtime < 0) {
		deadtime_val = conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_TIMER_CLOCK);
	} else {
		deadtime_val = conf_general_calculate_deadtime(deadtime, SYSTEM_TIMER_CLOCK);
	}

	// Automatic Output enable, Break, dead time and lock configuration
	TIM1->BDTR |= (deadtime_val & 0xFF) | TIM_BDTR_OSSR | TIM_BDTR_OSSI | TIM_BDTR_BKP;

	// Enable Capture Compare Preload Control (CCPC bit)
	TIM1->CR2 |= TIM_CR2_CCPC;

	// Enable Auto-Reload Preload Enable (ARPE bit)
	TIM1->CR1 |= TIM_CR1_ARPE;

	// Reset counter to zero
	TIM1->CNT = 0;

	// Set SMS = 0b110 (Trigger Mode)
	TIM1->SMCR |= (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1);  // SMS bits 2 and 1 set, bit 0 cleared

	// Set TS = 0b011 (Internal Trigger 3)
	TIM1->SMCR |= (TIM_SMCR_TS_1 | TIM_SMCR_TS_0);  // TS bits 1 and 0 set, bit 2 cleared

	// 3. Select One Pulse Mode
	TIM1->CR1 |= TIM_CR1_OPM;

	// Generate an update event to load preload registers immediately
	TIM1->EGR = TIM_EGR_UG;

	// 4. Enable Main Outputs
	TIM1->BDTR |= TIM_BDTR_MOE;

	// Enable TIM1 Counter
	TIM1->CR1 |= TIM_CR1_CEN;

	// Enable TIM4 Counter
	TIM4->CR1 |= TIM_CR1_CEN;

	// Disable TIM4 Counter immediately
	TIM4->CR1 &= ~TIM_CR1_CEN;

	// Configure TIM1 ARR and CCR1
	TIM1->ARR = (breaktime + pulse2) * utick;
	TIM1->CCR1 = breaktime * utick;

	// Wait for pulse 1
	while (TIM1->CNT != 0);

	// Pulse 2
	TIM4->CR1 |= TIM_CR1_CEN;

	// Wait for pulse 2
	chThdSleepMilliseconds(10);

	// Disable TIM1 Main Outputs
	TIM1->BDTR &= ~TIM_BDTR_MOE;

	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	switch (mcconf->motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_init(mcconf);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_init(mcconf, mcconf);
		break;
	
	default:
		break;
	}
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
		PAL_STM32_OSPEED_HIGHEST |
		PAL_STM32_PUPDR_FLOATING);
	commands_printf("Done");
	mempools_free_mcconf(mcconf);
	return;

}

void hw_init_gpio(void) {


	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// gpio
	palSetPadMode(PIN_TEST_GPIO, PIN_TEST_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOD, 9,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	PIN_TEST_OFF();
	palClearPad(GPIOD, 9);

	// ShutDown
	palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN, PAL_MODE_INPUT);
	HW_SHUTDOWN_HOLD_ON();

	// LSM6DS3 INT1 - not used yet, just kept out of a floating state.
	// NSS/SCK/MISO/MOSI are left alone here: imu_init_lsm6ds3_spi() configures
	// them for hardware SPI1 when the IMU is actually initialized.
	palSetPadMode(LSM6DS3_INT1_GPIO, LSM6DS3_INT1_PIN, PAL_MODE_INPUT_PULLDOWN);

//	INIT_BR();

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT);

	// Phase filters
	palSetPadMode(PHASE_FILTER_GPIO_PIN1, PHASE_FILTER_PIN1, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(PHASE_FILTER_GPIO_PIN2, PHASE_FILTER_PIN2, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(PHASE_FILTER_GPIO_PIN3, PHASE_FILTER_PIN3, PAL_MODE_OUTPUT_OPENDRAIN);
	PHASE_FILTER_OFF();
	
	// Current filter
	palSetPadMode(CURRENT_FILTER_GPIO_PIN1, CURRENT_FILTER_PIN1, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(CURRENT_FILTER_GPIO_PIN2, CURRENT_FILTER_PIN2, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(CURRENT_FILTER_GPIO_PIN3, CURRENT_FILTER_PIN3, PAL_MODE_OUTPUT_OPENDRAIN);
	CURRENT_FILTER_OFF();

	// I2C
    palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
        PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
        PAL_STM32_OTYPE_OPENDRAIN |
		PAL_STM32_OSPEED_MID1);

    palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
        PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
		PAL_STM32_OTYPE_OPENDRAIN |
		PAL_STM32_OSPEED_MID1);

	// put imu to i2c mode
	palSetPadMode(PIN_TEST_GPIO, PIN_TEST_PIN,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST);

	palSetPadMode(PIN_TEST_GPIO, PIN_TEST_PIN,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST);


	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);


	terminal_register_command_callback(
		"double_pulse",
		"Start a double pulse test",
		0,
		terminal_cmd_doublepulse);	

	terminal_register_command_callback(
		"test_button",
		"Try sampling the shutdown button",
		0,
		terminal_button_test);

	terminal_register_command_callback(
		"shutdown",
		"Shutdown VESC now.",
		0,
		terminal_shutdown_now);

	// Sensor port voltage
//	SENSOR_PORT_3V3();
//	palSetPadMode(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN,
//			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// ZCD-pin
//	palSetPadMode(ZCD_GPIO, ZCD_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//	palClearPad(ZCD_GPIO, ZCD_PIN);

	// CAN_EN-pin
//	palSetPadMode(CAN_EN_GPIO, CAN_EN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//	palClearPad(CAN_EN_GPIO, CAN_EN_PIN);

	// ADC Pins


//	// DAC as voltage reference for shunt amps
//	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
//	rccEnableDAC1(TRUE);
//	DAC1->CR |= DAC_CR_EN1;
//	DAC1->DHR12R1 = 2047;

}

void hw_setup_adc_channels(void) {
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG);      // PA7 - IN7 - CURR1         - SLOW - Reacts to PA7
    palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);      // PB1 - IN5 - SENS1         - FAST - Reacts to PB1
    palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);      // PA6 - IN3 - EXT1         - FAST - Reacts to PA6
    palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);      // PC5 - IN8 - TEMP_MOTOR    - SLOW - Reacts to PC5

	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);      // PA3 - IN15 - CURR2        - SLOW - Reacts to PA3
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);      // PA2 - IN14 - SENS2        - SLOW - Reacts to PA2
    palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);      // PB0 - IN9 - TEMP_MOS    - SLOW - Reacts to PB0
    palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);      // PC4 - IN4 - EXT2         - FAST - Reacts to PC4

    palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);      // PC1 - IN11 - CURR3        - SLOW - Reacts to PC1
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);      // PC0 - IN10 - SENS3        - SLOW - Reacts to PC0
    palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);      // PC2 - IN0 - VIN            - FAST - Reacts to PC2
    palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);      // PC3 - IN1 - EXT3        - FAST - Reacts to PC3

    // ADC1
    hw_setup_adc_channel_helper(ADC1, 7, 1, ADC_SMPR_SMP_64P5); //CURR1 3
	hw_setup_adc_channel_helper(ADC1, 5, 2, ADC_SMPR_SMP_64P5); //SENS1 0
    hw_setup_adc_channel_helper(ADC1, 8, 3, ADC_SMPR_SMP_64P5); //TEMP MOT 6
    hw_setup_adc_channel_helper(ADC1, 3, 4, ADC_SMPR_SMP_64P5); //EXT 9
	ADC1->PCSEL = ADC_PCSEL_PCSEL_5 | ADC_PCSEL_PCSEL_7 | ADC_PCSEL_PCSEL_8 | ADC_PCSEL_PCSEL_3;

    // ADC2
    hw_setup_adc_channel_helper(ADC2, 15, 1, ADC_SMPR_SMP_64P5); //CURR2 4
	hw_setup_adc_channel_helper(ADC2, 14, 2, ADC_SMPR_SMP_64P5); //SENS2 1
    hw_setup_adc_channel_helper(ADC2, 9, 3, ADC_SMPR_SMP_64P5); //TEMP MOS 7
    hw_setup_adc_channel_helper(ADC2, 4, 4, ADC_SMPR_SMP_64P5); // EXT2 10
	ADC2->PCSEL = ADC_PCSEL_PCSEL_14 | ADC_PCSEL_PCSEL_15 | ADC_PCSEL_PCSEL_9 | ADC_PCSEL_PCSEL_4;

    // ADC3
    hw_setup_adc_channel_helper(ADC3, 11, 1, ADC_SMPR_SMP_64P5); //CURR3 5
	hw_setup_adc_channel_helper(ADC3, 10, 2, ADC_SMPR_SMP_64P5); //SENS3 2
    hw_setup_adc_channel_helper(ADC3, 0, 3, ADC_SMPR_SMP_64P5); //VIN 8
    hw_setup_adc_channel_helper(ADC3, 1, 4, ADC_SMPR_SMP_64P5); // EXT3 11
	ADC3->PCSEL = ADC_PCSEL_PCSEL_10 | ADC_PCSEL_PCSEL_11 | ADC_PCSEL_PCSEL_0 | ADC_PCSEL_PCSEL_1;

/*
	hw_setup_inj_adc_channel_helper(ADC1, 7, 1, ADC_SMPR_SMP_16P5);   // CURR1
	hw_setup_inj_adc_channel_helper(ADC2, 15, 1, ADC_SMPR_SMP_16P5);  // CURR2
	hw_setup_inj_adc_channel_helper(ADC3, 11, 1, ADC_SMPR_SMP_16P5);  // CURR3
	hw_setup_inj_adc_channel_helper(ADC1, 7, 2, ADC_SMPR_SMP_16P5);   // CURR1
	hw_setup_inj_adc_channel_helper(ADC2, 15, 2, ADC_SMPR_SMP_16P5);  // CURR2
	hw_setup_inj_adc_channel_helper(ADC3, 11, 2, ADC_SMPR_SMP_16P5);  // CURR3
*/
}



void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}


/**
 * Try to restore the i2c bus
 */

void hw_try_restore_i2c(void) {
	if (i2c_running) {
		hw_stop_i2c();
		rccDisableI2C1();
		rccResetI2C1();

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		rccEnableI2C1(false);
		hw_start_i2c();
	}
}

bool hw_i2c_tx_rx(uint8_t dev_addr,
                  const uint8_t *tx_buf, size_t tx_len,
                  uint8_t *rx_buf, size_t rx_len)
{
    bool res = false;
    msg_t status = MSG_OK;

    i2cAcquireBus(&HW_I2C_DEV);

    // Transmit only
    if (tx_len > 0 && rx_len == 0) {
        status = i2cMasterTransmit(&HW_I2C_DEV,
                                   dev_addr,
                                   tx_buf, tx_len,
                                   NULL, 0);

        if (status != MSG_OK) {
            commands_printf(
                "I2C TX FAIL addr=0x%02X status=%d",
                dev_addr, status
            );
        }

        res = (status == MSG_OK);
    }
    // Receive only
    else if (tx_len == 0 && rx_len > 0) {
        status = i2cMasterReceive(&HW_I2C_DEV,
                                  dev_addr,
                                  rx_buf, rx_len);

        if (status != MSG_OK) {
            commands_printf(
                "I2C RX FAIL addr=0x%02X status=%d",
                dev_addr, status
            );
        }

        res = (status == MSG_OK);
    }
    // Combined TX -> RX
    else if (tx_len > 0 && rx_len > 0) {
        status = i2cMasterTransmitTimeout(&HW_I2C_DEV,
                                          dev_addr,
                                          tx_buf, tx_len,
                                          rx_buf, rx_len,
                                          TIME_MS2I(10));

        if (status != MSG_OK) {
            commands_printf(
                "I2C TXRX FAIL addr=0x%02X status=%d tx_len=%u rx_len=%u",
                dev_addr,
                status,
                (unsigned)tx_len,
                (unsigned)rx_len
            );
			i2cflags_t err = i2cGetErrors(&HW_I2C_DEV);
			commands_printf("I2C error flags: 0x%08lx", (uint32_t)err);
        }

        res = (status == MSG_OK);
    }
    else {
        commands_printf(
            "I2C INVALID CALL addr=0x%02X tx_len=%u rx_len=%u",
            dev_addr,
            (unsigned)tx_len,
            (unsigned)rx_len
        );
    }

    i2cReleaseBus(&HW_I2C_DEV);
    return res;
}

/**
 * Force the LSM6DS3 to select the I2C protocol on its shared NSS/SCK/MISO/MOSI
 * pins. Per the datasheet the chip latches its interface from the CS/NSS pin
 * state: NSS held high through the first access after power-up selects I2C,
 * while a CS-low SPI transaction latches SPI mode until the next power cycle.
 * There's no external pull-up on NSS on this board, so this has to be driven
 * explicitly before the I2C bit-bang fallback in imu_init_lsm6ds3() runs.
 */
void hw_lsm6ds3_force_i2c_mode(void) {
	palSetPadMode(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN);
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 10;i++) {
		commands_printf("BT: %d", (palReadPad(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN) == PAL_HIGH));
		chThdSleepMilliseconds(100);
	}
}

static void terminal_shutdown_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	DISABLE_GATE();
	HW_SHUTDOWN_HOLD_OFF();
}

/**
 * hw_sample_shutdown_button - instantaneous sample of the shutdown button state.
 *
 * HW_SHUTDOWN_SENSE_PIN is a plain digital input with an external pulldown that
 * reads high while the button is physically held and low otherwise - no ADC,
 * no on-die debounce needed here. shutdown.c's generic shutdown_thread already
 * majority-votes ~70 calls to this function over a 700ms window to debounce and
 * detect a press-release click, so this just needs to return a faithful,
 * repeatable level sample each call - it must NOT be stateful/one-shot, or the
 * vote in shutdown.c can never see a stable result.
 *
 * The one bit of state kept here is a boot-time guard: if this board's power-on
 * sequence is driven by holding this same physical button (common pattern - an
 * external latch circuit powers the board on while held, and firmware only
 * takes over from there), the sense pin can still read high for the tail end of
 * that press when this function starts getting called. Reporting "not pressed"
 * until we've seen at least one real low reading prevents that trailing release
 * from being misread as an immediate shutdown click right after power-on.
 */
static bool m_initial_release_seen = false;

bool hw_sample_shutdown_button(void) {
    bool is_high = (palReadPad(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN) == PAL_HIGH);

    if (!m_initial_release_seen) {
        if (!is_high) {
            m_initial_release_seen = true;
        }
        return false;
    }

    return is_high;
}