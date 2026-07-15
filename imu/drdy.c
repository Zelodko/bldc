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

#include "drdy.h"
#include "conf_general.h"

#ifdef IMU_DRDY_GPIO

#include "hal.h"

static const ioline_t m_line = PAL_LINE(IMU_DRDY_GPIO, IMU_DRDY_PIN);

static binary_semaphore_t m_sem;
static volatile bool m_sem_ready = false;
static volatile uint32_t m_int_count;
static volatile uint32_t m_timeout_count;

bool drdy_present(void) {
	return true;
}

static void drdy_pal_cb(void *arg) {
	(void)arg;
	drdy_signal_isr();
}

void drdy_init(void) {
	chBSemObjectInit(&m_sem, true); // start taken
	m_int_count = 0;
	m_timeout_count = 0;
	m_sem_ready = true;

	palSetPadMode(IMU_DRDY_GPIO, IMU_DRDY_PIN, PAL_MODE_INPUT_PULLDOWN);

	// The shared EXTI9_5/EXTI10_15 (or EXTI0..4) vector covering this pin is already enabled
	// unconditionally at halInit() (STM32H7xx/stm32_isr.c); only this line's own event needs
	// arming here.
	palSetLineCallback(m_line, drdy_pal_cb, NULL);
	palEnableLineEvent(m_line, PAL_EVENT_MODE_RISING_EDGE);
}

void drdy_deinit(void) {
	m_sem_ready = false;

	// Disables only this line's event; the shared vector stays enabled for other lines.
	palDisableLineEvent(m_line);
}

bool drdy_wait(systime_t timeout) {
	if (chBSemWaitTimeout(&m_sem, timeout) == MSG_TIMEOUT) {
		m_timeout_count++;
		return false;
	}
	return true;
}

void drdy_signal(void) {
	if (m_sem_ready) {
		chBSemSignal(&m_sem);
	}
}

void drdy_signal_isr(void) {
	m_int_count++;
	chSysLockFromISR();
	if (m_sem_ready) {
		chBSemSignalI(&m_sem);
	}
	chSysUnlockFromISR();
}

uint32_t drdy_interrupt_count(void) {
	return m_int_count;
}

uint32_t drdy_timeout_count(void) {
	return m_timeout_count;
}

#else // no DRDY pin wired on this board

bool drdy_present(void) {
	return false;
}

void drdy_init(void) {
}

void drdy_deinit(void) {
}

bool drdy_wait(systime_t timeout) {
	(void)timeout;
	return false;
}

void drdy_signal(void) {
}

void drdy_signal_isr(void) {
}

uint32_t drdy_interrupt_count(void) {
	return 0;
}

uint32_t drdy_timeout_count(void) {
	return 0;
}

#endif
