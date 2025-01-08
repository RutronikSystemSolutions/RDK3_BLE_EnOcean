/*
 * hal_i2c.c
 *
 *  Created on: 24 Mar 2023
 *      Author: jorda
 */

#include "hal_i2c.h"

#include "cyhal_i2c.h"
#include "cycfg_pins.h"

#include <stdlib.h>

/**
 * @var i2c_master Global variable storing the instance of the I2C master module
 */
static cyhal_i2c_t i2c_master;

static const uint32_t STD_TIMEOUT_MS = 100;


/**
 * @brief Init the I2C communication
 */
int8_t hal_i2c_init()
{
	cyhal_i2c_cfg_t i2c_config;
	i2c_config.is_slave = false;
	i2c_config.address = 0;
	i2c_config.frequencyhal_hz = 400000UL;

	cy_rslt_t result = cyhal_i2c_init(&i2c_master, ARDU_SDA, ARDU_SCL, NULL);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_i2c_configure(&i2c_master, &i2c_config);
	return result;
}

int8_t hal_i2c_read(uint8_t address, uint8_t* data, uint16_t len)
{
	cy_rslt_t result = cyhal_i2c_master_read(&i2c_master, (uint16_t)address, data, len, STD_TIMEOUT_MS, true);
	if (result != CY_RSLT_SUCCESS) return -1;
	return 0;
}

int8_t hal_i2c_write(uint8_t address, uint8_t* data, uint16_t len)
{
	cy_rslt_t result = cyhal_i2c_master_write(&i2c_master, (uint16_t)address, data, len, STD_TIMEOUT_MS, true);
	if (result != CY_RSLT_SUCCESS) return -1;
	return 0;
}

int8_t hal_i2c_write_register(uint8_t address, uint8_t reg, uint8_t* data, uint16_t size)
{
	cy_rslt_t result;
	uint8_t* i2c_data = NULL;

	// Allocate buffer for a register and the data
    i2c_data = malloc(size+1);
    if(i2c_data == NULL) return 1;

    // Copy register register and all the data
    i2c_data[0] = reg;
    memcpy(&i2c_data[1], data, size);

    // Execute write command
    result = cyhal_i2c_master_write( &i2c_master, (uint16_t)address, i2c_data, size+1, STD_TIMEOUT_MS, true );

    // Free allocated buffer and exit
    free(i2c_data);

    if (result != CY_RSLT_SUCCESS) return -1;
    return 0;
}

int8_t hal_i2c_read_register(uint8_t address, uint8_t reg, uint8_t* data, uint16_t size)
{
	cy_rslt_t result;

    result = cyhal_i2c_master_write( &i2c_master, (uint16_t)address, &reg, 1, STD_TIMEOUT_MS, false );
    if (result != CY_RSLT_SUCCESS) return -1;

    result = cyhal_i2c_master_read( &i2c_master, (uint16_t)address, data, size, STD_TIMEOUT_MS, true );

    if (result != CY_RSLT_SUCCESS) return -2;
    return 0;
}
