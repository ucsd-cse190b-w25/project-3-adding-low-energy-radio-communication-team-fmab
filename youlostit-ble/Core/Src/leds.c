/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
	/* Configure PA5 as an output by clearing all bits and setting the mode */

	//setting clock thing
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

	/* Configure the GPIO to use very high speed mode */
	GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

	/* Turn off the LED */
	GPIOA->ODR &= ~GPIO_ODR_OD5;



	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= GPIO_MODER_MODE14_0;

	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

	/* Configure the GPIO to use very high speed mode */
	GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

	/* Turn off the LED */
	GPIOB->ODR &= ~GPIO_ODR_OD14;


}

void leds_set(uint8_t led)
{
  // TODO implement this
	//passing in 0(0b00) means that both are off, 1(0b01) means led1 is on, 2(0b10) means led2 is on, 3(0b11) means both are on
	if (led == 0) {
		//turn off both


		/* Turn off the LED */
		GPIOA->ODR &= ~GPIO_ODR_OD5;

		//turn off led2 (PB14)
		GPIOB->ODR &= ~GPIO_ODR_OD14;
	}
	if (led == 1) {
		//turn on led1 (PA5)


		/* Turn on the LED */
		GPIOA->ODR |= GPIO_ODR_OD5;

		//turn off led2 (PB14)
		GPIOB->ODR &= ~GPIO_ODR_OD14;
	}

	if (led == 2) {
		//turn on led2 (PB14)


		/* Turn off the LED1 */
		GPIOA->ODR &= ~GPIO_ODR_OD5;

		//turn on led2 (PB14)
		GPIOB->ODR |= GPIO_ODR_OD14;
	}

	if (led == 3) {
		//turn on both


		/* Turn on the LED1 */
		GPIOA->ODR |= GPIO_ODR_OD5;

		//turn on led2 (PB14)
		GPIOB->ODR |= GPIO_ODR_OD14;
	}

}
