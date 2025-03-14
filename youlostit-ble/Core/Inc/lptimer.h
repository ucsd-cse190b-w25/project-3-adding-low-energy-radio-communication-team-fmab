/*
 * lptimer.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Kela
 */

#ifndef LPTIMER_H_
#define LPTIMER_H_

#include "stm32l4xx.h"  // Adjust this if you're using a different STM32 family

void lptimer_init(LPTIM_TypeDef* lptimer);
//void lptimer_reset(LPTIM_TypeDef* lptimer);
void lptim_set_ms(uint32_t ms);
//void LPTIM1_IRQHandler(void);

#endif /* INC_LPTIMER_H_ */
