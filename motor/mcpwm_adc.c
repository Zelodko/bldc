#include "mcpwm_adc.h"
#include "mc_interface.h"
#include "hal.h"
#include "hw.h"

#include <string.h>

// Shared STM32H7 ADC and DMA ownership for the mutually exclusive FOC and
// BLDC motor-control implementations.
#define ADC_DMA_MODE (STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | \
		STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PL(3))
// Bound all hardware-ready polling so a failed ADC cannot hang initialization.
#define ADC_WAIT_LOOPS (STM32_SYS_CK / 1000U)
#define ADC_REG_TRIG_TIM2_OC2 3U
#define ADC_REG_TRIG_TIM8_TRGO 7U
#define ADC_INJ_TRIG_TIM1_OC4 1U
#define ADC_INJ_TRIG_TIM2_OC1 3U
#define ADC_INJ_TRIG_TIM8_TRGO2 10U

static const stm32_dma_stream_t *dma_streams[3];
static bool adc_initialized;
static mcpwm_adc_mode_t adc_mode;

static bool wait_set(volatile uint32_t *reg, uint32_t mask) {
	uint32_t timeout = ADC_WAIT_LOOPS;
	while (!(*reg & mask) && timeout) {
		timeout--;
	}
	return timeout != 0;
}

static bool wait_clear(volatile uint32_t *reg, uint32_t mask) {
	uint32_t timeout = ADC_WAIT_LOOPS;
	while ((*reg & mask) && timeout) {
		timeout--;
	}
	return timeout != 0;
}

static void release_dma(void) {
	for (unsigned i = 0; i < 3; i++) {
		if (dma_streams[i]) {
			dmaStreamDisable(dma_streams[i]);
			dmaStreamFreeI(dma_streams[i]);
			dma_streams[i] = NULL;
		}
	}
}

static void cleanup(void) {
	adc_initialized = false;
	nvicDisableVector(ADC_IRQn);
	ADC1->IER = 0;
	ADC2->IER = 0;
	ADC3->IER = 0;
	rccResetADC12();
	rccResetADC3();
	release_dma();
}

static unsigned regular_channel(ADC_TypeDef *adc, unsigned rank) {
	return (adc->SQR1 >> (rank * 6U)) & 31U;
}

static uint32_t injected_sequence(unsigned channel, unsigned trigger) {
	return (trigger << ADC_JSQR_JEXTSEL_Pos) | ADC_JSQR_JEXTEN_1 |
			(channel << ADC_JSQR_JSQ1_Pos);
}

bool mcpwm_adc_init(mcpwm_adc_mode_t mode, mcpwm_adc_callback_t callback) {
	if (dma_streams[0] || dma_streams[1] || dma_streams[2]) {
		return false;
	}
	// The BLDC injected-trigger arrangement has only been qualified on rev V.
	if (mode == MCPWM_ADC_MODE_BLDC && (DBGMCU->IDCODE >> 16) != 0x2003U) {
		return false;
	}

	// In BLDC mode, stream 3 is serviced last among equal-priority streams,
	// so its completion callback sees the complete three-ADC regular frame.
	unsigned callback_stream = mode == MCPWM_ADC_MODE_FOC ? 0U : 2U;
	dma_streams[0] = dmaStreamAllocI(STM32_DMA_STREAM_ID(1, 1), 5,
			callback_stream == 0U ? (stm32_dmaisr_t)callback : NULL, NULL);
	dma_streams[1] = dmaStreamAllocI(STM32_DMA_STREAM_ID(1, 2), 5, NULL, NULL);
	dma_streams[2] = dmaStreamAllocI(STM32_DMA_STREAM_ID(1, 3), 5,
			callback_stream == 2U ? (stm32_dmaisr_t)callback : NULL, NULL);
	if (!dma_streams[0] || !dma_streams[1] || !dma_streams[2]) {
		release_dma();
		return false;
	}

	rccResetADC12();
	rccResetADC3();
	rccEnableADC12(true);
	rccEnableADC3(true);

	ADC_TypeDef *adcs[] = {ADC1, ADC2, ADC3};
	volatile uint16_t *buffers[] = {
		&ADC_Value[HW_ADC_IND_DMA_1],
		&ADC_Value[HW_ADC_IND_DMA_2],
		&ADC_Value[HW_ADC_IND_DMA_3]
	};
	const uint32_t requests[] = {
		STM32_DMAMUX1_ADC1,
		STM32_DMAMUX1_ADC2,
		STM32_DMAMUX1_ADC3
	};

	for (unsigned i = 0; i < 3; i++) {
		dmaStreamSetPeripheral(dma_streams[i], &adcs[i]->DR);
		dmaStreamSetMemory0(dma_streams[i], buffers[i]);
		dmaStreamSetTransactionSize(dma_streams[i], HW_ADC_NBR_CONV);
		dmaStreamSetMode(dma_streams[i], ADC_DMA_MODE |
				(i == callback_stream ? STM32_DMA_CR_TCIE : 0U));
		dmaSetRequestSource(dma_streams[i], requests[i]);
	}

	for (unsigned i = 0; i < 3; i++) {
		adcs[i]->CR = ADC_CR_ADVREGEN;
	}
	osalSysPolledDelayX(OSAL_US2RTC(STM32_SYS_CK, 20U));
	for (unsigned i = 0; i < 3; i++) {
		if (!wait_set(&adcs[i]->ISR, ADC_ISR_LDORDY)) {
			cleanup();
			return false;
		}
		adcs[i]->CR |= ADC_CR_BOOST | ADC_CR_ADCAL;
	}
	for (unsigned i = 0; i < 3; i++) {
		if (!wait_clear(&adcs[i]->CR, ADC_CR_ADCAL)) {
			cleanup();
			return false;
		}
	}

	uint32_t regular_trigger = mode == MCPWM_ADC_MODE_FOC ?
			ADC_REG_TRIG_TIM2_OC2 : ADC_REG_TRIG_TIM8_TRGO;
	for (unsigned i = 0; i < 3; i++) {
		adcs[i]->CFGR = ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1 |
				ADC_CFGR_RES_1 | ADC_CFGR_RES_2 |
				(regular_trigger << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_1;
		adcs[i]->SQR1 = (HW_ADC_NBR_CONV - 1U) << ADC_SQR1_L_Pos;
	}

	hw_setup_adc_channels();
	if (mode == MCPWM_ADC_MODE_BLDC) {
		unsigned current_ranks[] = {
			ADC_IND_CURR1 - HW_ADC_IND_DMA_1 + 1U,
			ADC_IND_CURR2 - HW_ADC_IND_DMA_2 + 1U,
			ADC_IND_CURR3 - HW_ADC_IND_DMA_3 + 1U
		};
		for (unsigned i = 0; i < 3; i++) {
			if (current_ranks[i] < 1U || current_ranks[i] > HW_ADC_NBR_CONV) {
				cleanup();
				return false;
			}
		}
		unsigned current_channels[] = {
			regular_channel(ADC1, current_ranks[0]),
			regular_channel(ADC2, current_ranks[1]),
			regular_channel(ADC3, current_ranks[2])
		};
		// A fixed, queue-disabled context avoids the revision-V JSQR queue errata.
		ADC1->CFGR |= ADC_CFGR_JQDIS;
		ADC2->CFGR |= ADC_CFGR_JQDIS;
		ADC3->CFGR |= ADC_CFGR_JQDIS;
		ADC1->JSQR = injected_sequence(current_channels[0], ADC_INJ_TRIG_TIM1_OC4);
		ADC2->JSQR = injected_sequence(current_channels[1], ADC_INJ_TRIG_TIM8_TRGO2);
		ADC3->JSQR = injected_sequence(current_channels[2], ADC_INJ_TRIG_TIM2_OC1);
		ADC1->IER = ADC_IER_JEOSIE;
	}

	for (unsigned i = 0; i < 3; i++) {
		adcs[i]->ISR = ADC_ISR_ADRDY | ADC_ISR_OVR | ADC_ISR_JEOC |
				ADC_ISR_JEOS | ADC_ISR_JQOVF;
		adcs[i]->CR |= ADC_CR_ADEN;
	}
	for (unsigned i = 0; i < 3; i++) {
		if (!wait_set(&adcs[i]->ISR, ADC_ISR_ADRDY)) {
			cleanup();
			return false;
		}
	}

	for (unsigned i = 0; i < 3; i++) {
		dmaStreamEnable(dma_streams[i]);
		adcs[i]->CR |= ADC_CR_ADSTART;
		if (mode == MCPWM_ADC_MODE_BLDC) {
			adcs[i]->CR |= ADC_CR_JADSTART;
		}
	}
	if (mode == MCPWM_ADC_MODE_BLDC) {
		nvicEnableVector(ADC_IRQn, 6);
	}
	adc_mode = mode;
	adc_initialized = true;

	return true;
}

void mcpwm_adc_deinit(void) {
	osalSysLock();
	cleanup();
	osalSysUnlock();
}

void mcpwm_adc_get_status(mcpwm_adc_status_t *status) {
	memset(status, 0, sizeof(*status));

	osalSysLock();
	status->initialized = adc_initialized;
	status->mode = adc_mode;

	ADC_TypeDef *adcs[] = {ADC1, ADC2, ADC3};
	for (unsigned i = 0; i < 3; i++) {
		status->adc_cr[i] = adcs[i]->CR;
		status->adc_ier[i] = adcs[i]->IER;
		status->adc_cfgr[i] = adcs[i]->CFGR;
		status->adc_sqr1[i] = adcs[i]->SQR1;
		status->adc_jsqr[i] = adcs[i]->JSQR;

		if (dma_streams[i]) {
			status->dma_cr[i] = dma_streams[i]->stream->CR;
			status->dma_ndtr[i] = dma_streams[i]->stream->NDTR;
			status->dma_par[i] = dma_streams[i]->stream->PAR;
			status->dma_m0ar[i] = dma_streams[i]->stream->M0AR;
			status->dmamux_ccr[i] = dma_streams[i]->mux->CCR;
		}
	}
	osalSysUnlock();
}
