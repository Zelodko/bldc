/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "hw.h"
#include "encoder/encoder.h"
#include "main.h"

__attribute__((section(".itcm_text")))
CH_IRQ_HANDLER(ADC1_2_3_IRQHandler) {
	CH_IRQ_PROLOGUE();
	if ((ADC1->ISR & ADC_ISR_JEOS) && (ADC1->IER & ADC_IER_JEOSIE)) {
		mc_interface_adc_inj_int_handler();
		ADC1->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
		ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
		ADC3->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
	}
	CH_IRQ_EPILOGUE();
}

__attribute__((section(".itcm_text")))
CH_IRQ_HANDLER(TIM2_IRQHandler) {
	if((TIM2->SR & TIM_SR_CC2IF) && (TIM2->DIER & TIM_DIER_CC2IE)){
		mcpwm_foc_tim_sample_int_handler();

		// Clear the IT pending bit
		TIM2->SR = ~TIM_SR_CC2IF;
	}
	// Clear the IT pending bit
	TIM2->SR = ~TIM_SR_CC2IF;
}

CH_IRQ_HANDLER(NMI_Handler) {
	main_stop_motor_and_reset();
}

CH_IRQ_HANDLER(HardFault_Handler) {
	main_stop_motor_and_reset();
}

CH_IRQ_HANDLER(MemManage_Handler) {
	main_stop_motor_and_reset();
}

CH_IRQ_HANDLER(BusFault_Handler) {
	main_stop_motor_and_reset();
}

CH_IRQ_HANDLER(UsageFault_Handler) {
	main_stop_motor_and_reset();
}

// Called from ChibiOS's shared EXTI16 handler (see STM32_EXTI16_ISR in
// hwconf/mcuconf-h7.h). Supply voltage dropped below the PVD threshold
// (~2.85V/2.75V rising/falling), which could corrupt an ongoing flash
// programming operation.
void pvd_exti_isr(uint32_t pr, uint32_t line) {
	if ((pr & EXTI_MASK1(line)) != 0) {
		mc_interface_fault_stop(FAULT_CODE_MCU_UNDER_VOLTAGE, false, true);
	}
}
