/**
 ******************************************************************************
 * @file    EEPROM/EEPROM_Emulation/src/eeprom.c
 * @author  MCD Application Team
 * @brief   This file provides all the EEPROM emulation firmware functions.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/** @addtogroup EEPROM_Emulation
 * @{
 */

#ifdef EEPROM_HOST_TEST
#include "eeprom_test_hal.h"
#else
#include "eeprom.h"
#define EE_READ32(a) (*(volatile uint32_t *)(a))
#endif

/* Keep the legacy 64-byte header/record spacing. Each 32-byte flash word is
 * programmed once per erase. A separate commit word makes rollover recoverable.
 * A receiving page is never used by normal reads or writes. */
#define EE_MAGIC 0x32454548U
#define EE_COMMIT 0x43454548U
#define EE_RECORD_SIZE 64U
#define EE_FAST __attribute__((section(".itcm_text")))
extern uint16_t VirtAddVarTab[NB_OF_VAR];
static uint32_t active_page;
static uint32_t generation;
static bool legacy_page;
static bool ready;

static uint32_t other_page(uint32_t page) {
	return page == PAGE0_BASE_ADDRESS ? PAGE1_BASE_ADDRESS : PAGE0_BASE_ADDRESS;
}

static bool erased(uint32_t address, uint32_t bytes) {
	for (uint32_t i = 0; i < bytes; i += 4) {
		if (EE_READ32(address + i) != 0xFFFFFFFFU) { return false; }
	}
	return true;
}

static EE_FAST uint16_t program(uint32_t address, const uint32_t *data) {
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	SCB_DisableICache();
	uint16_t result = HAL_FLASH_Program(address, (const uint8_t *)data, 32);
	SCB_EnableICache();
	SCB_EnableDCache();
	__set_PRIMASK(mask);
	if (result == FLASH_NO_ERROR) {
		for (unsigned i = 0; i < 8; i++) {
			if (EE_READ32(address + i * 4) != data[i]) { return FLASH_ERROR_VERIFY; }
		}
	}
	return result;
}

static EE_FAST uint16_t erase_page(uint32_t page) {
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	SCB_CleanInvalidateDCache();
	SCB_DisableDCache();
	SCB_DisableICache();
	uint16_t result = HAL_FLASH_Erase(FLASH_BANK_2,
			page == PAGE0_BASE_ADDRESS ? PAGE0_ID : PAGE1_ID, 1);
	SCB_EnableICache();
	SCB_EnableDCache();
	__set_PRIMASK(mask);
	return result;
}

static bool committed(uint32_t page, uint32_t *seq) {
	uint32_t n = EE_READ32(page + 4);
	if (EE_READ32(page) != EE_MAGIC || EE_READ32(page + 8) != ~n ||
			EE_READ32(page + 32) != EE_COMMIT ||
			EE_READ32(page + 36) != n || EE_READ32(page + 40) != ~n) {
		return false;
	}
	*seq = n;
	return true;
}

static bool legacy(uint32_t page, uint16_t status) {
	if (EE_READ32(page) != status || !erased(page + 32, 32)) { return false; }
	for (unsigned i = 4; i < 32; i += 4) {
		if (EE_READ32(page + i) != 0) { return false; }
	}
	return true;
}

static uint16_t read_page(uint32_t page, uint16_t key, uint16_t *value) {
	for (uint32_t offset = PAGE_SIZE - EE_RECORD_SIZE; offset >= EE_RECORD_SIZE;
			offset -= EE_RECORD_SIZE) {
		uint32_t address = page + offset;
		/* Legacy records have zero padding; checking the full key word avoids
		 * treating an erased/torn address word as a committed record. */
		if (EE_READ32(address + 32) == key) {
			*value = (uint16_t)EE_READ32(address);
			return 0;
		}
	}
	return 1;
}

static EE_FAST uint16_t append(uint32_t page, uint16_t key, uint16_t value) {
	for (uint32_t offset = EE_RECORD_SIZE; offset < PAGE_SIZE; offset += EE_RECORD_SIZE) {
		uint32_t address = page + offset;
		if (!erased(address, EE_RECORD_SIZE)) { continue; }
		uint32_t line[8] = {value};
		uint16_t result = program(address, line);
		if (result != FLASH_NO_ERROR) { return result; }
		line[0] = key;
		return program(address + 32, line);
	}
	return PAGE_FULL;
}

static EE_FAST uint16_t begin_page(uint32_t page, uint32_t seq) {
	uint16_t result = erase_page(page);
	if (result != FLASH_NO_ERROR) { return result; }
	uint32_t line[8] = {EE_MAGIC, seq, ~seq};
	return program(page, line);
}

static EE_FAST uint16_t commit_page(uint32_t page, uint32_t seq) {
	uint32_t line[8] = {EE_COMMIT, seq, ~seq};
	return program(page + 32, line);
}

static EE_FAST uint16_t transfer(uint16_t key, uint16_t value) {
	uint32_t destination = other_page(active_page);
	uint32_t seq = generation + 1U;
	uint16_t result = begin_page(destination, seq);
	if (result != FLASH_NO_ERROR) { return result; }
	if (key != 0xFFFFU) {
		result = append(destination, key, value);
		if (result != FLASH_NO_ERROR) { return result; }
	}
	for (unsigned i = 0; i < NB_OF_VAR; i++) {
		uint16_t k = VirtAddVarTab[i], v, existing;
		if (!k || k == 0xFFFFU || k == key) { continue; }
		if (read_page(active_page, k, &v) != 0) { continue; }
		if (read_page(destination, k, &existing) == 0) { continue; }
		result = append(destination, k, v);
		if (result != FLASH_NO_ERROR) { return result; }
	}
	/* Every programmed line was read back before reaching the commit. Keep
	 * the source intact, even after commit; erase it only on the next rollover.
	 * Generation numbers disambiguate two committed pages after reset. */
	// A controller error may still have programmed the commit. Until a fresh
	// initialization resolves that state, never acknowledge writes to the old page.
	ready = false;
	result = commit_page(destination, seq);
	if (result != FLASH_NO_ERROR) { return result; }
	active_page = destination;
	generation = seq;
	legacy_page = false;
	ready = true;
	return FLASH_NO_ERROR;
}

EE_FAST uint16_t EE_Init(void) {
	ready = false;
	active_page = 0;
	generation = 0;
	legacy_page = false;
	uint32_t seq0 = 0, seq1 = 0;
	bool valid0 = committed(PAGE0_BASE_ADDRESS, &seq0);
	bool valid1 = committed(PAGE1_BASE_ADDRESS, &seq1);
	if (valid0 || valid1) {
		if (valid0 && valid1 && (seq0 == seq1 || seq0 - seq1 == 0x80000000U)) {
			return NO_VALID_PAGE; // Ambiguous: preserve flash for diagnosis.
		}
		bool use0 = valid0 && (!valid1 || (int32_t)(seq0 - seq1) > 0);
		active_page = use0 ? PAGE0_BASE_ADDRESS : PAGE1_BASE_ADDRESS;
		generation = use0 ? seq0 : seq1;
	} else {
		valid0 = legacy(PAGE0_BASE_ADDRESS, VALID_PAGE);
		valid1 = legacy(PAGE1_BASE_ADDRESS, VALID_PAGE);
		if (valid0 && valid1) { return NO_VALID_PAGE; }
		if (!valid0 && !valid1) {
			/* Old firmware erased its source before promoting RECEIVE_DATA.
			 * Only accept that legacy state when the other sector is erased. */
			valid0 = legacy(PAGE0_BASE_ADDRESS, RECEIVE_DATA) && erased(PAGE1_BASE_ADDRESS, PAGE_SIZE);
			valid1 = legacy(PAGE1_BASE_ADDRESS, RECEIVE_DATA) && erased(PAGE0_BASE_ADDRESS, PAGE_SIZE);
		}
		if (valid0 || valid1) {
			active_page = valid0 ? PAGE0_BASE_ADDRESS : PAGE1_BASE_ADDRESS;
			legacy_page = true;
		} else if ((erased(PAGE0_BASE_ADDRESS, PAGE_SIZE) ||
				(EE_READ32(PAGE0_BASE_ADDRESS) == EE_MAGIC &&
				 erased(PAGE0_BASE_ADDRESS + EE_RECORD_SIZE, PAGE_SIZE - EE_RECORD_SIZE))) &&
				erased(PAGE1_BASE_ADDRESS, PAGE_SIZE)) {
			// Also recover interruption during the first empty-page initialization.
			uint16_t result = begin_page(PAGE0_BASE_ADDRESS, 0);
			if (result == FLASH_NO_ERROR) { result = commit_page(PAGE0_BASE_ADDRESS, 0); }
			if (result != FLASH_NO_ERROR) { return result; }
			active_page = PAGE0_BASE_ADDRESS;
		} else {
			return NO_VALID_PAGE; // Never silently format nonempty unknown data.
		}
	}
	ready = true;
	return FLASH_NO_ERROR;
}

uint16_t EE_ReadVariable(uint16_t key, uint16_t *value) {
	if (!ready) { return NO_VALID_PAGE; }
	return read_page(active_page, key, value);
}

EE_FAST uint16_t EE_WriteVariable(uint16_t key, uint16_t value) {
	if (!ready) { return NO_VALID_PAGE; }
	bool known = false;
	for (unsigned i = 0; i < NB_OF_VAR; i++) {
		if (VirtAddVarTab[i] == key && key && key != 0xFFFFU) { known = true; break; }
	}
	if (!known) { return FLASH_ERROR_PROGRAM; }
	uint16_t old;
	if (read_page(active_page, key, &old) == 0 && old == value) { return FLASH_NO_ERROR; }
	if (legacy_page) { return transfer(key, value); }
	uint16_t result = append(active_page, key, value);
	return result == PAGE_FULL ? transfer(key, value) : result;
}
