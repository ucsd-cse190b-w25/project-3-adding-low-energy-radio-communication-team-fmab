/*
 * i2c.h
 *
 *  Created on: Jan 27, 2025
 *      Author: phil
 */

#ifndef I2C_H_
#define I2C_H_


void i2c_init();
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);


#endif /* I2C_H_ */
