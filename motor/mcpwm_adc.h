#ifndef MCPWM_ADC_H_
#define MCPWM_ADC_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	MCPWM_ADC_MODE_FOC,
	MCPWM_ADC_MODE_BLDC
} mcpwm_adc_mode_t;

typedef void (*mcpwm_adc_callback_t)(void *param, uint32_t flags);

typedef struct {
	bool initialized;
	mcpwm_adc_mode_t mode;
	uint32_t adc_cr[3];
	uint32_t adc_ier[3];
	uint32_t adc_cfgr[3];
	uint32_t adc_sqr1[3];
	uint32_t adc_jsqr[3];
	uint32_t dma_cr[3];
	uint32_t dma_ndtr[3];
	uint32_t dma_par[3];
	uint32_t dma_m0ar[3];
	uint32_t dmamux_ccr[3];
} mcpwm_adc_status_t;

// Initialization is called with the ChibiOS system lock held. The status
// snapshot lets the motor implementation reject a partial hardware bring-up.
bool mcpwm_adc_init(mcpwm_adc_mode_t mode, mcpwm_adc_callback_t callback);
void mcpwm_adc_deinit(void);
void mcpwm_adc_get_status(mcpwm_adc_status_t *status);

#endif
