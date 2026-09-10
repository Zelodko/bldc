/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HW_G200H7_V2_CORE_H_
#define HW_G200H7_V2_CORE_H_

#define HW_NAME					"G200H7_V2"

// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_PHASE_FILTERS
// The gate driver is permanently enabled and has no fault or timer-break
// signal routed to the MCU. Safe shutdown therefore relies on timer outputs.

// Macros
#define LED_GREEN_GPIO			GPIOE
#define LED_GREEN_PIN			3
#define LED_RED_GPIO			GPIOE
#define LED_RED_PIN				2
#define PIN_TEST_GPIO			GPIOD
#define PIN_TEST_PIN			8

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PIN_TEST_ON()			palSetPad(PIN_TEST_GPIO, PIN_TEST_PIN)
#define PIN_TEST_OFF()			palClearPad(PIN_TEST_GPIO, PIN_TEST_PIN)

#define PIN_ADCTEST_ON()			palSetPad(GPIOD, 9)
#define PIN_ADCTEST_OFF()			palClearPad(GPIOD, 9)

#define PHASE_FILTER_GPIO_PIN1		GPIOE
#define PHASE_FILTER_GPIO_PIN2		GPIOA
#define PHASE_FILTER_GPIO_PIN3		GPIOE
#define PHASE_FILTER_PIN1		13
#define PHASE_FILTER_PIN2		5
#define PHASE_FILTER_PIN3		6

#define PHASE_FILTER_ON() 		palClearPad(PHASE_FILTER_GPIO_PIN1, PHASE_FILTER_PIN1);palClearPad(PHASE_FILTER_GPIO_PIN2, PHASE_FILTER_PIN2);palClearPad(PHASE_FILTER_GPIO_PIN3, PHASE_FILTER_PIN3);
#define PHASE_FILTER_OFF()      palSetPad(PHASE_FILTER_GPIO_PIN1, PHASE_FILTER_PIN1);palSetPad(PHASE_FILTER_GPIO_PIN2, PHASE_FILTER_PIN2);palSetPad(PHASE_FILTER_GPIO_PIN3, PHASE_FILTER_PIN3);

#define CURRENT_FILTER_GPIO_PIN1		GPIOB
#define CURRENT_FILTER_GPIO_PIN2		GPIOE
#define CURRENT_FILTER_GPIO_PIN3		GPIOA
#define CURRENT_FILTER_PIN1		12
#define CURRENT_FILTER_PIN2		9
#define CURRENT_FILTER_PIN3		0

#define CURRENT_FILTER_ON()		palClearPad(CURRENT_FILTER_GPIO_PIN1, CURRENT_FILTER_PIN1);palClearPad(CURRENT_FILTER_GPIO_PIN2, CURRENT_FILTER_PIN2);palClearPad(CURRENT_FILTER_GPIO_PIN3, CURRENT_FILTER_PIN3);
#define CURRENT_FILTER_OFF()	palSetPad(CURRENT_FILTER_GPIO_PIN1, CURRENT_FILTER_PIN1);palSetPad(CURRENT_FILTER_GPIO_PIN2, CURRENT_FILTER_PIN2);palSetPad(CURRENT_FILTER_GPIO_PIN3, CURRENT_FILTER_PIN3);
//#define CURRENT_FILTER_OFF()		palClearPad(CURRENT_FILTER_GPIO_PIN1, CURRENT_FILTER_PIN1);palClearPad(CURRENT_FILTER_GPIO_PIN2, CURRENT_FILTER_PIN2);palClearPad(CURRENT_FILTER_GPIO_PIN3, CURRENT_FILTER_PIN3);

// LSM6DS3 - wired to hardware SPI1 (NSS=PA15, SCK=PB3, MISO=PB4, MOSI=PB5), with a
// data-ready interrupt on INT1=PC13. The LSM6DS3 multiplexes I2C onto the SPI pins
// (SCK doubles as SCL, MOSI as SDA); IMU_FALLBACK_COM is the I2C bit-bang path taken
// if the hardware-SPI probe in imu_init() fails.
#define IMU_DEV					IMU_DEV_LSM6DS3
#define IMU_COM					IMU_COM_SPI_HW
#define IMU_SPI_DEV				SPID1
#define IMU_SPI_AF				GPIO_AF_SPI1
#define IMU_SPI_NSS_GPIO		GPIOA
#define IMU_SPI_NSS_PIN			15
#define IMU_SPI_SCK_GPIO		GPIOB
#define IMU_SPI_SCK_PIN			3
#define IMU_SPI_MISO_GPIO		GPIOB
#define IMU_SPI_MISO_PIN		4
#define IMU_SPI_MOSI_GPIO		GPIOB
#define IMU_SPI_MOSI_PIN		5

#define IMU_FALLBACK_COM			IMU_COM_I2C_BB
#define IMU_FALLBACK_I2C_SCL_GPIO	GPIOB
#define IMU_FALLBACK_I2C_SCL_PIN	3
#define IMU_FALLBACK_I2C_SDA_GPIO	GPIOB
#define IMU_FALLBACK_I2C_SDA_PIN	5

#define IMU_DRDY_GPIO			GPIOC
#define IMU_DRDY_PIN			13

// ADC setup
#define HW_ADC_NBR_CONV      4
#define HW_ADC_CHANNELS      12
#define HW_ADC_INJ_CHANNELS  3

// DMA base offsets
#define HW_ADC_IND_DMA_1     0
#define HW_ADC_IND_DMA_2     4
#define HW_ADC_IND_DMA_3     8


#define ADC_IND_CURR1        0   // ADC1 (IN7)
#define ADC_IND_SENS1        1   // ADC1 (IN5)
#define ADC_IND_TEMP_MOTOR   2   // ADC1 (IN8)
#define ADC_IND_EXT          3   // ADC1 (IN3)

#define ADC_IND_CURR2        4   // ADC2 (IN15)
#define ADC_IND_SENS2        5   // ADC2 (IN14)
#define ADC_IND_TEMP_MOS     6   // ADC2 (IN9)
#define ADC_IND_EXT2         7   // ADC2 (IN4)

#define ADC_IND_CURR3        8   // ADC3 (IN11)
#define ADC_IND_SENS3        9   // ADC3 (IN10)
#define ADC_IND_VIN_SENS     10  // ADC3 (IN0)
#define ADC_IND_EXT3         11  // ADC3 (IN1)


// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					330000.0
#endif
#ifndef VIN_R2
#define VIN_R2					5560.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.304
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0001667
#endif

#define ADC_VOLTS_INPUT_FACTOR	        1.0135

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2)) * ADC_VOLTS_INPUT_FACTOR

// NTC Termistors
#define NTC_RES(adc_val) (((adc_val) < 1 || (adc_val) > 4095.0) ? 10000.0 : ((4095.0 * 10000.0) / (adc_val) - 10000.0))
#define NTC_TEMP(adc_ind) (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			6
#define HW_ADC_EXT2_GPIO		GPIOC
#define HW_ADC_EXT2_PIN			4

// Shutdown pin
#define HW_SHUTDOWN_GPIO		    GPIOD
#define HW_SHUTDOWN_PIN			    5
#define HW_SHUTDOWN_SENSE_GPIO		GPIOE
#define HW_SHUTDOWN_SENSE_PIN		4
#define HW_SHUTDOWN_HOLD_ON()		palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN); PIN_TEST_OFF();
#define HW_SHUTDOWN_HOLD_OFF()		palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN); PIN_TEST_ON();
#define HW_SAMPLE_SHUTDOWN()		hw_sample_shutdown_button()

// UART Peripheral
#define HW_UART_DEV				SD5
#define HW_UART_GPIO_AF			GPIO_AF_UART5
#define HW_UART_TX_PORT			GPIOC
#define HW_UART_TX_PIN			12
#define HW_UART_RX_PORT			GPIOD
#define HW_UART_RX_PIN			2

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		rccEnableTIM4(TRUE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOD
#define HW_ICU_PIN				12
#define HW_ICU_RESET()			rccResetTIM4()
/*
// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

*/
// Permanent I2C (for internal IMU)
#define HW_I2C_DEV              I2CD1
#define HW_I2C_GPIO_AF          GPIO_AF_I2C1
#define HW_I2C_SCL_PORT		    GPIOB
#define HW_I2C_SCL_PIN          8
#define HW_I2C_SDA_PORT         GPIOB
#define HW_I2C_SDA_PIN          7

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		rccEnableTIM3(TRUE)
#define HW_ENC_EXTI_PORTSRC		GPIOC
#define HW_ENC_EXTI_PINSRC		8
#define HW_ENC_TIM_RESET()		rccResetTIM3()

// SPI pins
#define HW_SPI_DEV				SPID3
#define HW_SPI_GPIO_AF			GPIO_AF_SPI3
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOC
#define HW_SPI_PIN_SCK			10
#define HW_SPI_PORT_MOSI		GPIOC
#define HW_SPI_PIN_MOSI			12
#define HW_SPI_PORT_MISO		GPIOC
#define HW_SPI_PIN_MISO			11



// NRF SWD
#define NRF5x_SWDIO_GPIO		GPIOD
#define NRF5x_SWDIO_PIN			9
#define NRF5x_SWCLK_GPIO		GPIOD
#define NRF5x_SWCLK_PIN			13

// Permanent UART
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_DEV_TX		SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOD
#define HW_UART_P_TX_PIN		1
#define HW_UART_P_RX_PORT		GPIOA
#define HW_UART_P_RX_PIN		1

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

#define HW_DEAD_TIME_NSEC		20.0


// Default setting overrides
#define MCCONF_L_MIN_VOLTAGE			    60.0		// Minimum input voltage

#define MCCONF_L_MAX_VOLTAGE			    170.0	// Maximum input voltage

#define MCCONF_L_BATTERY_CUT_START		    65.0	// Start limiting the positive current at this voltage

#define MCCONF_L_BATTERY_CUT_END		    60.0		// Limit the positive current completely at this voltage

#define MCCONF_L_BATTERY_REGEN_CUT_START    160.0	// Start limiting the regen current at this voltage

#define MCCONF_L_BATTERY_REGEN_CUT_END      170.0		// Limit the regen current completely at this voltage

#define MCCONF_DEFAULT_MOTOR_TYPE		    MOTOR_TYPE_FOC

#define MCCONF_FOC_F_ZV					    32000.0

#define MCCONF_L_MAX_ABS_CURRENT		    100.0	// The maximum absolute current above which a fault is generated

#define MCCONF_L_CURRENT_MAX			    50.0	// Current limit in Amperes (Upper)

#define MCCONF_L_CURRENT_MIN			    -50.0	 // Current limit in Amperes (Lower)

#define MCCONF_L_SLOW_ABS_OVERCURRENT	    false	// Use the raw current for the overcurrent fault detection

#define MCCONF_L_IN_CURRENT_MAX			    5.0	// Input current limit in Amperes (Upper)

#define MCCONF_L_IN_CURRENT_MIN			    -5.0	// Input current limit in Amperes (Lower)

#define MCCONF_L_MIN_DUTY				    0.005	// Minimum duty cycle

#define MCCONF_L_MAX_DUTY				    0.90	// Maximum duty cycle

#define MCCONF_L_LIM_TEMP_FET_START		    40.0	// MOSFET temperature where current limiting should begin

#define MCCONF_L_LIM_TEMP_FET_END		    60.0	// MOSFET temperature where everything should be shut off

#define MCCONF_M_MOTOR_TEMP_SENS_TYPE       TEMP_SENSOR_DISABLED // Motor Temperature Sensor Type

#define MCCONF_FOC_CURRENT_SAMPLE_MODE	    FOC_CURRENT_SAMPLE_MODE_ALL_SENSORS

#define MCCONF_FOC_DT_US				    0.02    // Microseconds for dead time compensation

#define MCCONF_BMS_TYPE					    BMS_TYPE_NONE

#define MCCONF_FOC_OFFSETS_CAL_MODE         0   // Don't Measure offsets every boot, it is done once at motor setup

#define MCCONF_FOC_PHASE_FILTER_DISABLE_FAULT	false

#define APPCONF_SHUTDOWN_MODE               SHUTDOWN_MODE_TOGGLE_BUTTON_ONLY

#define APPCONF_CAN_BAUD_RATE				CAN_BAUD_1M


//#define MCCONF_FOC_OFFSETS_CURRENT_0	33158.0 // Current 0 offset
//#define MCCONF_FOC_OFFSETS_CURRENT_1	33030.0 // Current 1 offset
//define MCCONF_FOC_OFFSETS_CURRENT_2	32983.0 // Current 2 offset

//#define CURRENT_CAL1				    0.915
//#define CURRENT_CAL2				    0.952
//#define CURRENT_CAL3				    0.92



// Setting limits
#define HW_LIM_CURRENT			-200.0, 200.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 300.0
#define HW_LIM_VIN				0.0, 180.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 90.0
#define HW_LIM_FOC_CTRL_LOOP_FREQ	16000.0, 64000.0
#define HW_LIM_FOC_F_ZV			16000.0, 64000.0
#define HW_LIM_BLDC_F_SW		8000.0, 35000.0
#define HW_LIM_DC_F_SW			8000.0, 35000.0
// BLDC is operational with synchronous PWM, but hardware validation is still
// limited. Keep the other BLDC PWM modes and DC motor mode disabled until they
// are qualified separately on this power stage. The upstream BLDC parameter
// detector uses a 50% spin-up target, so detection needs at least 0.5 max duty.
#define HW_BLDC_FORCE_PWM_MODE	PWM_MODE_SYNCHRONOUS
#define HW_DISABLE_DC_MOTOR_MODE

//functions
bool hw_sample_shutdown_button(void);


#endif /* HW_G200H7_V2_CORE_H_ */
