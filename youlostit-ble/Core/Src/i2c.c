/*
 * i2c.c
 *
 *  Created on: Jan 27, 2025
 *      Author: phil
 */
#include <stdint.h>
#include <stm32l475xx.h>
#include <stdio.h>
#include "i2c.h"

void i2c_init() {
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;   // Enable I2C2 clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   //enable GPIOB clock

	I2C2->CR1 &= ~I2C_CR1_PE; // Disable the I2C2 peripheral for configuration
	//repeat above for PB11 pin

	GPIOB->MODER &= ~(3 << (10 * 2));  // Clear PB10 mode
	GPIOB->MODER |= (2 << (10 * 2));   // Set PB10 to Alternate Function mode (10)

	GPIOB->MODER &= ~(3 << (11 * 2));  // Clear PB11 mode
	GPIOB->MODER |= (2 << (11 * 2)); 	//Set PB11 to Alternate Function mode (10)

	 // setting to Open-Drain mode for PB10 and PB11

	GPIOB->OTYPER |= (1 << 10) | (1 << 11);   //set bit 10 and 11 to 1, which sets to open drain

	GPIOB->PUPDR &= ~(3 << (10 * 2));  // Clear PB10 PUPDR
	GPIOB->PUPDR |= (1 << (10 * 2));   // Set PB10 as Pull-up (01)

	GPIOB->PUPDR &= ~(3 << (11 * 2));  // Clear PB11 PUPDR
	GPIOB->PUPDR |= (1 << (11 * 2));    //Set PB11 as Pull-up (01)




	GPIOB->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL10_Pos);  //clear AF bits
	GPIOB->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL11_Pos);
	GPIOB->AFR[1] |= (0x4 << GPIO_AFRH_AFSEL10_Pos) | (0x4 << GPIO_AFRH_AFSEL11_Pos);



	I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_SDADEL | I2C_TIMINGR_PRESC);

	//I2C2->TIMINGR |= (0 << I2C_TIMINGR_PRESC_Pos);

	//I2C2->TIMINGR &= ~I2C_TIMINGR_SCLL;
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);

	//I2C2->TIMINGR &= ~I2C_TIMINGR_SCLH;
	I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);

	//I2C2->TIMINGR &= ~I2C_TIMINGR_SDADEL;
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);

	//I2C2->TIMINGR &= ~I2C_TIMINGR_SCLDEL;
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);





	// Enable I2C2 peripheral
	I2C2->CR1 |= I2C_CR1_PE; // Enable the I2C2 peripheral



}
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len){

	while (I2C2->ISR & I2C_ISR_BUSY);



	I2C2->CR2 &= ~I2C_CR2_NBYTES;
	I2C2->CR2 &= ~I2C_CR2_SADD;
	I2C2->CR2 &= ~I2C_CR2_START;


	if (dir == 0) {  // WRITE OPERATION


		//I2C2->CR2 = 0;
		I2C2->CR2 |= (address << 1);
		I2C2->CR2 |= (0 << I2C_CR2_RD_WRN_Pos);
		I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos);

		I2C2->CR2 |= I2C_CR2_START;

		for (uint8_t i = 0; i < len; i++) {

			while ((I2C2->ISR & I2C_ISR_NACKF) || (!(I2C2->ISR & I2C_ISR_TXIS)));  // Wait until TX buffer is empty

			I2C2->TXDR = data[i] & 0xFF;  // Send data byte
			//printf("data ith elem w %x \n", data[i]);
		}
		while (!(I2C2->ISR & I2C_ISR_TC));  // Wait until transfer is complete
	}

	else {  // READ OPERATION



		I2C2->CR2 |= (address << 1);
		I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos);
		I2C2->CR2 |= (1 << I2C_CR2_RD_WRN_Pos);
		I2C2->CR2 |= I2C_CR2_START;

		for (uint8_t i = 0; i < len; i++) {
			while (!(I2C2->ISR & I2C_ISR_RXNE));  // Wait until RX buffer is full
			data[i] = I2C2->RXDR & 0xFF;  // Read data byte
			//printf("data ith elem r %x \n", data[i]);
		}
		while (!(I2C2->ISR & I2C_ISR_TC));  // Wait until transfer is complete

	}

	I2C2->CR2 |= I2C_CR2_STOP;
	I2C2->CR2 = 0x0000;

	return 0;
}

