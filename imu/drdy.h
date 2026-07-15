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

#ifndef IMU_DRDY_H_
#define IMU_DRDY_H_

#include "ch.h"

#include <stdint.h>
#include <stdbool.h>

// Generic data-ready (DRDY) EXTI source. A board opts in by defining IMU_DRDY_GPIO and
// IMU_DRDY_PIN. With neither defined every function below is an inert stub and
// drdy_present() is false, so the IMU thread stays in its timed loop.
//
// (Upstream's F4 drdy.c additionally needs IMU_DRDY_EXTI_PORTSRC/_PINSRC/_LINE, to feed
// SYSCFG_EXTILineConfig()/EXTI_InitTypeDef by hand. This port instead uses ChibiOS's native
// palEnableLineEvent()/palSetLineCallback(), which derive the EXTI channel straight from the
// GPIO port/pad, so those extra macros aren't needed here.)

// True iff this board wires a DRDY pin (compile-time constant).
bool drdy_present(void);

// Arm the DRDY pin's EXTI line (the shared vector is enabled centrally, at halInit()). No-op
// when absent.
void drdy_init(void);

// Mask the DRDY line again, leaving the shared vector enabled. No-op when absent.
void drdy_deinit(void);

// Block until the next data-ready edge or until timeout elapses. Returns true if an edge was
// signalled, false on timeout (counted). Returns false immediately when absent.
bool drdy_wait(systime_t timeout);

// Release a waiter from thread context (e.g. to unblock the loop for shutdown).
void drdy_signal(void);

// Release a waiter from the EXTI callback.
void drdy_signal_isr(void);

uint32_t drdy_interrupt_count(void);
uint32_t drdy_timeout_count(void);

#endif /* IMU_DRDY_H_ */
