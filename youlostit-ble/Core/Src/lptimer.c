/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


extern volatile int lostFlag;  //0 means not lost, 1 means lost

void lptimer_init(TIM_TypeDef* timer)
{
  // TODO implement this

 	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;   // Enable TIM2 clock
 	RCC->BDCR |= RCC_BDCR_LSEON;
 	while(!(RCC->BDCR & RCC_BDCR_LSERDY));
 	RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
 	//RCC->APB1SMENR1 |= RCC_APB1SMENR1_LPTIM1EN;
	timer_reset(timer);     //reset counter
	LPTIM1->CFGR = LPTIM_CFGR_PRESC_5;
	/*if(lostFlag == 0) {
		//its at 100khz so prescale
		//timer->PSC = 99;    //scales down to 1khz

		timer_set_ms(timer, 500000);   //sets autoreload to 50, so that it reloads every 1/20th of a second, 20Hz
	}
	else{   //its lost so its at 8mhz*/
	timer->PSC = 31;    //scales down to 1khz

	timer_set_ms(timer, 5000);   //sets autoreload to 50, so that it reloads every 1/20th of a second, 20Hz


	LPTIM1->IER |= LPTIM_IER_ARRMIE;
	//->CR1 &= ~TIM_CR1_DIR;    //set timer to up counting mode 50



    // 6. Enable the timer interrupt for update events
    TIM2->DIER |= TIM_DIER_UIE;  // Enable update interrupt


    // 7. Enable the timer interrupt in the NVIC
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 1);  // Set priority to 1 (lower number = higher priority)

    // 8. Start the timer
    TIM2->CR1 |= TIM_CR1_CEN;   // Enable the timer (start counting)



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


