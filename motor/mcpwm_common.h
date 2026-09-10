/*
	Copyright 2016 - 2025 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MCPWM_COMMON_H_
#define MCPWM_COMMON_H_

// Common macros

#ifdef INVERTED_TOP_DRIVER_INPUT
#define TIMER_TOP_POLARITY (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P)
#define TIMER_TOP_IDLE_STATE (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3)
#else
#define TIMER_TOP_POLARITY 0U
#define TIMER_TOP_IDLE_STATE 0U
#endif

#ifdef INVERTED_BOTTOM_DRIVER_INPUT
#define TIMER_BOTTOM_POLARITY (TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP)
#define TIMER_BOTTOM_IDLE_STATE (TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N)
#else
#define TIMER_BOTTOM_POLARITY 0U
#define TIMER_BOTTOM_IDLE_STATE 0U
#endif

#define TIMER_OUTPUT_POLARITY (TIMER_TOP_POLARITY | TIMER_BOTTOM_POLARITY)
#define TIMER_OUTPUT_IDLE_STATE (TIMER_TOP_IDLE_STATE | TIMER_BOTTOM_IDLE_STATE)

// Decode the nonlinear STM32 advanced-timer DTG field into timer-clock ticks.
static inline uint32_t timer_deadtime_ticks(uint8_t dtg) {
	if ((dtg & 0x80U) == 0U) {
		return dtg;
	} else if ((dtg & 0xC0U) == 0x80U) {
		return (64U + (dtg & 0x3FU)) * 2U;
	} else if ((dtg & 0xE0U) == 0xC0U) {
		return (32U + (dtg & 0x1FU)) * 8U;
	}
	return (32U + (dtg & 0x1FU)) * 16U;
}

static inline uint8_t timer_deadtime_from_ns(float deadtime_ns, float timer_clock) {
	uint8_t dtg = conf_general_calculate_deadtime(deadtime_ns, timer_clock);
	float actual_ns = (float)timer_deadtime_ticks(dtg) * 1.0e9f / timer_clock;
	// Round upward when the generic conversion selected the lower adjacent step.
	if (actual_ns < deadtime_ns && dtg < 0xFFU) {
		dtg++;
	}
	return dtg;
}

////////////////////////////////////////////////////

#define TIMER_UPDATE_CH1_0() \
		TIM1->CCER &= ~TIM_CCER_CC1E; \
		TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; \
		TIM1->CCMR1 |= TIM_CCMR1_OC1M_2; \
		TIM1->CCER |= TIM_CCER_CC1E; \
		TIM1->CCER &= ~TIM_CCER_CC1NE;

#define TIMER_UPDATE_M2_CH1_0() \
		TIM8->CCER &= ~TIM_CCER_CC1E; \
		TIM8->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; \
		TIM8->CCMR1 |= TIM_CCMR1_OC1M_2; \
		TIM8->CCER |= TIM_CCER_CC1E; \
		TIM8->CCER &= ~TIM_CCER_CC1NE;

///

#define TIMER_UPDATE_CH1_POS() \
		TIM1->CCER &= ~TIM_CCER_CC1E; \
		TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; \
		TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; \
		TIM1->CCER |= TIM_CCER_CC1E; \
		TIM1->CCER |= TIM_CCER_CC1NE;

#define TIMER_UPDATE_M2_CH1_POS() \
		TIM8->CCER &= ~TIM_CCER_CC1E; \
		TIM8->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; \
		TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; \
		TIM8->CCER |= TIM_CCER_CC1E; \
		TIM8->CCER |= TIM_CCER_CC1NE;

///

#define TIMER_UPDATE_CH1_NEG() \
		TIM1->CCER &= ~TIM_CCER_CC1E; \
		TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; \
		TIM1->CCMR1 |= TIM_CCMR1_OC1M_2; \
		TIM1->CCER |= TIM_CCER_CC1E; \
		TIM1->CCER |= TIM_CCER_CC1NE;

#define TIMER_UPDATE_M2_CH1_NEG() \
		TIM8->CCER &= ~TIM_CCER_CC1E; \
		TIM8->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; \
		TIM8->CCMR1 |= TIM_CCMR1_OC1M_2; \
		TIM8->CCER |= TIM_CCER_CC1E; \
		TIM8->CCER |= TIM_CCER_CC1NE;

////////////////////////////////////////////////////

#define TIMER_UPDATE_CH2_0() \
		TIM1->CCER &= ~TIM_CCER_CC2E; \
		TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; \
		TIM1->CCMR1 |= TIM_CCMR1_OC2M_2; \
		TIM1->CCER |= TIM_CCER_CC2E; \
		TIM1->CCER &= ~TIM_CCER_CC2NE;

#define TIMER_UPDATE_M2_CH2_0() \
		TIM8->CCER &= ~TIM_CCER_CC2E; \
		TIM8->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; \
		TIM8->CCMR1 |= TIM_CCMR1_OC2M_2; \
		TIM8->CCER |= TIM_CCER_CC2E; \
		TIM8->CCER &= ~TIM_CCER_CC2NE;

///

#define TIMER_UPDATE_CH2_POS() \
		TIM1->CCER &= ~TIM_CCER_CC2E; \
		TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; \
		TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; \
		TIM1->CCER |= TIM_CCER_CC2E; \
		TIM1->CCER |= TIM_CCER_CC2NE;

#define TIMER_UPDATE_M2_CH2_POS() \
		TIM8->CCER &= ~TIM_CCER_CC2E; \
		TIM8->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; \
		TIM8->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; \
		TIM8->CCER |= TIM_CCER_CC2E; \
		TIM8->CCER |= TIM_CCER_CC2NE;

///

#define TIMER_UPDATE_CH2_NEG() \
		TIM1->CCER &= ~TIM_CCER_CC2E; \
		TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; \
		TIM1->CCMR1 |= TIM_CCMR1_OC2M_2; \
		TIM1->CCER |= TIM_CCER_CC2E; \
		TIM1->CCER |= TIM_CCER_CC2NE;

#define TIMER_UPDATE_M2_CH2_NEG() \
		TIM8->CCER &= ~TIM_CCER_CC2E; \
		TIM8->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; \
		TIM8->CCMR1 |= TIM_CCMR1_OC2M_2; \
		TIM8->CCER |= TIM_CCER_CC2E; \
		TIM8->CCER |= TIM_CCER_CC2NE;

////////////////////////////////////////////////////

#define TIMER_UPDATE_CH3_0() \
		TIM1->CCER &= ~TIM_CCER_CC3E; \
		TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; \
		TIM1->CCMR2 |= TIM_CCMR2_OC3M_2; \
		TIM1->CCER |= TIM_CCER_CC3E; \
		TIM1->CCER &= ~TIM_CCER_CC3NE;

#define TIMER_UPDATE_M2_CH3_0() \
		TIM8->CCER &= ~TIM_CCER_CC3E; \
		TIM8->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; \
		TIM8->CCMR2 |= TIM_CCMR2_OC3M_2; \
		TIM8->CCER |= TIM_CCER_CC3E; \
		TIM8->CCER &= ~TIM_CCER_CC3NE;

///

#define TIMER_UPDATE_CH3_POS() \
		TIM1->CCER &= ~TIM_CCER_CC3E; \
		TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; \
		TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; \
		TIM1->CCER |= TIM_CCER_CC3E; \
		TIM1->CCER |= TIM_CCER_CC3NE;

#define TIMER_UPDATE_M2_CH3_POS() \
		TIM8->CCER &= ~TIM_CCER_CC3E; \
		TIM8->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; \
		TIM8->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; \
		TIM8->CCER |= TIM_CCER_CC3E; \
		TIM8->CCER |= TIM_CCER_CC3NE;

///

#define TIMER_UPDATE_CH3_NEG() \
		TIM1->CCER &= ~TIM_CCER_CC3E; \
		TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; \
		TIM1->CCMR2 |= TIM_CCMR2_OC3M_2; \
		TIM1->CCER |= TIM_CCER_CC3E; \
		TIM1->CCER |= TIM_CCER_CC3NE;

#define TIMER_UPDATE_M2_CH3_NEG() \
		TIM8->CCER &= ~TIM_CCER_CC3E; \
		TIM8->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; \
		TIM8->CCMR2 |= TIM_CCMR2_OC3M_2; \
		TIM8->CCER |= TIM_CCER_CC3E; \
		TIM8->CCER |= TIM_CCER_CC3NE;

////////////////////////////////////////////////////

#define TIMER_CONTROL_UPDATE() (TIM1->EGR = TIM_EGR_COMG)
#define TIMER_M2_CONTROL_UPDATE() (TIM8->EGR = TIM_EGR_COMG)

////////////////////////////////////////////////////

#endif /* MCPWM_COMMON_H_ */
