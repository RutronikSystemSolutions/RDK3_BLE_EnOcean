/*
 * hal_i2c.h
 *
 *  Created on: 24 Mar 2023
 *      Author: jorda
 */

#ifndef HAL_HAL_I2C_H_
#define HAL_HAL_I2C_H_

#include <stdint.h>

int8_t hal_i2c_init();

int8_t hal_i2c_read(uint8_t address, uint8_t* data, uint16_t len);

int8_t hal_i2c_write(uint8_t address, uint8_t* data, uint16_t len);

int8_t hal_i2c_write_register(uint8_t address, uint8_t reg, uint8_t* data, uint16_t size);

int8_t hal_i2c_read_register(uint8_t address, uint8_t reg, uint8_t* data, uint16_t size);

#endif /* HAL_HAL_I2C_H_ */
