// Coherent phase-voltage snapshots for the VESC Tool sample logger.
#include "foc_sample.h"
#include "mc_interface.h"
#include "hal.h"
#include "hw.h"
#include <string.h>

#define FAST __attribute__((section(".itcm_text")))
#if defined(STM32H743xx)
#define DMA_ERRORS (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF | STM32_DMA_ISR_FEIF)
#endif

static struct {
	bool required, configured, valid;
	int voltage[3];
	uint32_t scan_ticks;
} snapshot;

void foc_sample_init(void) {
	memset(&snapshot, 0, sizeof(snapshot));
#if defined(STM32H743xx)
#if HW_ADC_NBR_CONV == 4 && HW_ADC_IND_DMA_1 == 0 && HW_ADC_IND_DMA_2 == 4 && HW_ADC_IND_DMA_3 == 8
#if !defined(HW_HAS_DUAL_MOTORS) && !defined(HW_HAS_DUAL_PARALLEL)
	snapshot.required = true;
	if ((DBGMCU->IDCODE >> 16) != 0x2003U || ADC12_COMMON->CCR || ADC3_COMMON->CCR ||
			(RCC->D3CCIPR & RCC_D3CCIPR_ADCSEL)) return;
	ADC_TypeDef *adcs[] = {ADC1, ADC2, ADC3};
	static const uint32_t sample_half_cycles[] = {3, 5, 17, 33, 65, 129, 775, 1621};
	uint32_t longest = 0;
	for (unsigned a = 0; a < 3; a++) {
		if ((adcs[a]->CFGR & ADC_CFGR_RES) != (ADC_CFGR_RES_2 | ADC_CFGR_RES_1) ||
				(adcs[a]->SQR1 & ADC_SQR1_L) != 3U) return;
		uint32_t cycles = 0;
		for (unsigned rank = 1; rank <= 4; rank++) {
			unsigned ch = (adcs[a]->SQR1 >> (rank * 6)) & 31U;
			if (ch > 19) return;
			uint32_t smpr = ch < 10 ? adcs[a]->SMPR1 : adcs[a]->SMPR2;
			cycles += sample_half_cycles[(smpr >> ((ch % 10) * 3)) & 7U] + 13U;
		}
		if (cycles > longest) longest = cycles;
	}
	// Revision V divides PLL2_P by two when the common prescaler is zero.
	snapshot.scan_ticks = ((uint64_t)longest * (SYSTEM_TIMER_CLOCK) + STM32_PLL2_P_CK - 1U) / STM32_PLL2_P_CK;
	snapshot.configured = true;
#endif
#endif
#endif
}

FAST void foc_sample_capture(uint32_t flags) {
	snapshot.valid = false;
	if (!snapshot.required || !mc_interface_sample_capture_active()) return;
#if defined(STM32H743xx)
	bool invalid = false;
	uint32_t start = TIM5->CNT;
	uint32_t phase = TIM2->CNT;
	uint32_t top = TIM1->ARR;
	if (!snapshot.configured) invalid = true;
	if (!(flags & STM32_DMA_ISR_TCIF) || (flags & DMA_ERRORS)) invalid = true;
	if ((ADC1->ISR | ADC2->ISR | ADC3->ISR) & ADC_ISR_OVR) invalid = true;
	if (phase < snapshot.scan_ticks || phase >= top) invalid = true;
	if (DMA1_Stream1->NDTR != HW_ADC_NBR_CONV || DMA1_Stream2->NDTR != HW_ADC_NBR_CONV ||
			DMA1_Stream3->NDTR != HW_ADC_NBR_CONV) invalid = true;
	__DMB();
	snapshot.voltage[0] = ADC_V_L1;
	snapshot.voltage[1] = ADC_V_L2;
	snapshot.voltage[2] = ADC_V_L3;
	__DMB();
	if (DMA1_Stream1->NDTR != HW_ADC_NBR_CONV || DMA1_Stream2->NDTR != HW_ADC_NBR_CONV ||
			DMA1_Stream3->NDTR != HW_ADC_NBR_CONV) invalid = true;
	// Reject reads that reach the next trigger, including complete TIM2 wraps.
	uint64_t elapsed = ((uint64_t)(uint32_t)(TIM5->CNT - start) + 1U) * (TIM5->PSC + 1U);
	if (phase >= top || elapsed >= top - phase) invalid = true;
	if (!invalid) {
		snapshot.valid = true;
	}
#else
	(void)flags;
#endif
}

FAST bool foc_sample_get_voltage(int voltage[3], bool is_second_motor) {
	if (!snapshot.required) {
#ifdef HW_HAS_DUAL_MOTORS
		if (is_second_motor) {
			voltage[0] = ADC_V_L4;
			voltage[1] = ADC_V_L5;
			voltage[2] = ADC_V_L6;
			return true;
		}
#else
		(void)is_second_motor;
#endif
		voltage[0] = ADC_V_L1;
		voltage[1] = ADC_V_L2;
		voltage[2] = ADC_V_L3;
		return true;
	}
	if (!snapshot.valid) return false;
	for (unsigned i = 0; i < 3; i++) voltage[i] = snapshot.voltage[i];
	return true;
}
