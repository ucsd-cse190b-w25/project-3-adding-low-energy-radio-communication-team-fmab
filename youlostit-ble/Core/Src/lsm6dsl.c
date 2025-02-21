/*
 * lsm6dsl.c
 *
 *  Created on: Jan 31, 2025
 *      Author: phil
 */
#include <stdint.h>
#include "lsm6dsl.h"
#include "i2c.h"
#include <stdio.h>
#include <stm32l475xx.h>

void lsm6dsl_init() {
	//writing to CTRL XL register in the LSM6DSR to configure register (address 0x6A, dir = 0, data= 0x60, len =8)



	//write to CTRL_XL register
	uint8_t CTRL1_XL_read;
	uint8_t CTRL1_XL = 0x10;   //address of ctrlXL
	uint8_t CTRL1_XL_DATA[2] = {0x10, 0x60};
	i2c_transaction(0x6A, 0, CTRL1_XL_DATA, 2);  //write



	//writing to INT1 CTRL register in the LSM6DSR to configure register (address 0x6A, dir = 0, data= 0x0D01, len =8)
	//printf("works here too\n");
	uint8_t INT1_CTRL_read;
	uint8_t INT1_CTRL = 0x0D;
	uint8_t INT1_CTRL_DATA[2] = {0x0D, 0x01};
	i2c_transaction(0x6A, 0, INT1_CTRL_DATA, 2);

	//printf("INT1_CTRL is %x \n", INT1_CTRL_read);
	//i2c_transaction(0x6A, 0, &INT1_CTRL, 1);

}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z){

	//for x low and x high
	uint8_t reg_addyXL = 0x28;   //register address (x low)
	uint8_t dataXL;   //declaring data array for the transaction function to read stuff into
	//repeat above for all register addresses

	i2c_transaction(0x6A, 0, &reg_addyXL, 1);
	i2c_transaction(0x6A, 1, &dataXL, 1);   //read in the 1 byte from x low

	uint8_t reg_addyXH = 0x29;   //register address (x high)
	uint8_t dataXH;   //declaring data array for the transaction function to read stuff into


	i2c_transaction(0x6A, 0, &reg_addyXH, 1);
	i2c_transaction(0x6A, 1, &dataXH, 1);   //read in the 1 byte from x high


	//for y low and y high

	uint8_t reg_addyYL = 0x2A;   //register address (y low)
	uint8_t dataYL;   //declaring data array for the transaction function to read stuff into

	i2c_transaction(0x6A, 0, &reg_addyYL, 1);
	i2c_transaction(0x6A, 1, &dataYL, 1);   //read in the 1 byte from y low


	uint8_t reg_addyYH = 0x2B;   //register address (y high)
	uint8_t dataYH;   //declaring data array for the transaction function to read stuff into

	i2c_transaction(0x6A, 0, &reg_addyYH, 1);
	i2c_transaction(0x6A, 1, &dataYH, 1);   //read in the 1 byte from y high


	//for z low and z high
	uint8_t reg_addyZL = 0x2C;   //register address   (z low)
	uint8_t dataZL;   //declaring data array for the transaction function to read stuff into

	i2c_transaction(0x6A, 0, &reg_addyZL, 1);
	i2c_transaction(0x6A, 1, &dataZL, 1);   //read in the 1 byte from z low


	uint8_t reg_addyZH = 0x2D;   //register address (z high)
	uint8_t dataZH;   //declaring data array for the transaction function to read stuff into

	i2c_transaction(0x6A, 0, &reg_addyZH, 1);
	i2c_transaction(0x6A, 1, &dataZH, 1);   //read in the 1 byte from z high



	*x = (dataXH << 8 | dataXL);
	*y = (dataYH << 8 | dataYL);
	*z = (dataZH << 8 | dataZL);
}

