/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
  // TODO implement this
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;   // Enable TIM2 clock
	timer_reset(timer);     //reset counter

	timer->PSC = 3999;    //scales down to 1khz 3999

	timer_set_ms(timer, 50);   //sets autoreload to 50, so that it reloads every 1/20th of a second, 20Hz
	TIM2->CR1 &= ~TIM_CR1_DIR;    //set timer to up counting mode 50



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


