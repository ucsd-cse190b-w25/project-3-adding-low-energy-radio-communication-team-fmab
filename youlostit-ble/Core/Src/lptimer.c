/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


extern volatile int lostFlag;  //0 means not lost, 1 means lost

void lptimer_init(LPTIM_TypeDef* lptimer) {
    // Enable the LPTIM2 clock
    //RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
	RCC->CIER |= RCC_CIER_LSIRDYIE;
	RCC->CSR |= RCC_CSR_LSION
	while (!(RCC->CSR & RCC_CSR_LSIRDY));
	RCC->APB!ENR1 |= rccapb1enr1lptime1en

			rcc ccipr &= ~RCC_CCIPR_LPTIME1sel
			RCC_> ccipr |= rcc_ccipr_ltptime1sel_0
			lptime1->cr &= ~lptim_cr_enable
			while (lptm1-> & lptim_cr_enable){}
	LPTIM1->ICR = LPTIMRRMCF
			LPTIMICR ARROKCF
			LPTIM ICR CMPOKCF
			LPTIMICR EXTTRIGCF
			CMPCF
			ARRMCF
			DOWNCF
	LPTIME1->CFGR = 0
	LPTIME10>CFGR = (0b101 << CFGR PRESC)

	LPTIME1->CNT = 0

	IER LPTIM IER ARRMIE

	NVIC SET PRIO LPTIM! IRQN 0}
NVIC ENABLE IRQ LPTIM! IRQN




}

void timer_reset(TIM_TypeDef* timer)
{
  // TODO implement this
	TIM2->CR1 &= ~TIM_CR1_CEN;  //stop the timer
	TIM2->CNT = 0x0000;  // Reset the counter to 0

}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  // TODO implement this

	timer->ARR = (period_ms) - 1;
}


