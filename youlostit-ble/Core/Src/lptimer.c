/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "lptimer.h"

void lptimer_init(LPTIM_TypeDef *timer)
{
    // Enable LPTIM1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
    lptimer_reset(timer);

    // Enable the LSI clock (32 kHz internal oscillator)
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));  // Wait for LSI to stabilize
    // Select LSI as the clock source for LPTIM1
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;  // Clear LPTIM1 clock source bits
    RCC->CCIPR |= (1 << RCC_CCIPR_LPTIM1SEL_Pos);  // Set LPTIM1 clock source to LSI (32 kHz)

    // Disable LPTIM1 before configuration
    timer->CR &= ~LPTIM_CR_ENABLE;

    // Configure LPTIM1 prescaler
    timer->CFGR &= ~LPTIM_CFGR_PRESC;  // Clear prescaler bits
    timer->CFGR |= (0b101 << LPTIM_CFGR_PRESC_Pos);  // Set prescaler to divide by 16 (32 kHz / 32 = 1 kHz)

    // Clear all interrupt flags
    timer->ICR |= LPTIM_ICR_ARRMCF | LPTIM_ICR_CMPOKCF | LPTIM_ICR_EXTTRIGCF | LPTIM_ICR_CMPMCF | LPTIM_ICR_DOWNCF;

    // Enable the autoreload match interrupt
    timer->IER |= LPTIM_IER_ARRMIE;

    // Enable LPTIM1 interrupt in the NVIC
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 0);

    // Enable LPTIM1
    timer->CR |= LPTIM_CR_ENABLE;


    // Start the timer in continuous mode
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}
void lptimer_reset(LPTIM_TypeDef* timer) {
	//timer->CR &= ~LPTIM_CR_ENABLE;  //stop the timer
	timer->CNT = 0x0000;  // Reset the counter to 0
}
void lptim_set_ms(uint32_t ms)
{

    // Clear any pending flags
    LPTIM1->ICR |= LPTIM_ICR_ARRMCF;

    // Write to the ARR register
    LPTIM1->ARR = ms - 1;

}
