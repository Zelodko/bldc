/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "timer.h"
#include "ch.h"
#include "hal.h"


// Settings
#define TIMER_HZ					1.4e7

void timer_init(void) {
	rccResetTIM5();

	rccEnableTIM5(TRUE);

	// Select the Counter Mode, UP (default)
	// Set the Autoreload value
	TIM5->ARR = 0xFFFFFFFF;
	// TIM5 is on APB1 (D2PPRE1). The F4 original divided SYSTEM_CORE_CLOCK (raw AHB
	// clock) by 2 here, which happened to equal TIM5's real kernel clock only because
	// F4's APB1 prescaler was /4 (AHB/4, then x2 timer-doubling = AHB/2). On this board,
	// SYSTEM_TIMER_CLOCK is already the post-doubling TIM1 kernel clock (240MHz), and
	// APB1/APB2 use the same /2 prescaler here, so TIM5's kernel clock equals
	// SYSTEM_TIMER_CLOCK directly - the extra /2 was double-counting the F4-specific
	// APB1/4 assumption and made TIM5 tick at ~2.14x the intended 14MHz, inflating every
	// timer_seconds_elapsed_since()/timer_calc_diff() result (incl. real control-loop dt
	// in the FOC PID/HFI threads) by the same ~2.14x.
	TIM5->PSC = (SYSTEM_TIMER_CLOCK / TIMER_HZ) - 1;
	TIM5->CNT = 0;
	// Update
	TIM5->EGR = TIM_EGR_UG;
	// Enable timer
	TIM5->CR1 |= TIM_CR1_CEN;
}

uint32_t timer_time_now(void) {
	return TIM5->CNT;
}

float timer_seconds_elapsed_since(uint32_t time) {
	uint32_t diff = TIM5->CNT - time;
	return (float)diff * (1.0 / (float)TIMER_HZ);
}

float timer_calc_diff(uint32_t start, uint32_t time) {
	uint32_t diff = time - start;
	return (float)diff * (1.0 / (float)TIMER_HZ);
}

/**
 * Blocking sleep based on timer.
 *
 * @param seconds
 * Seconds to sleep.
 */
void timer_sleep(float seconds) {
	uint32_t start_t = TIM5->CNT;

	for (;;) {
		if (timer_seconds_elapsed_since(start_t) >= seconds) {
			return;
		}
	}
}
