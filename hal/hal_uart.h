/*
 * hal_uart.h
 *
 *  Created on: 14 Nov 2023
 *      Author: jorda
 */

#ifndef HAL_HAL_UART_H_
#define HAL_HAL_UART_H_

#include <stdint.h>

/**
 * @brief Initialize the communication
 *
 * @retval 0 Success
 * @retval < 0 Error
 */
int hal_uart_init();

/**
 * @brief Get how much bytes are available in the UART buffer
 */
uint32_t hal_uart_readable(void);

/**
 * @brief Read from the UART module
 *
 * @param [out] buffer Buffer in which the read values will be stored
 * @param [in] size Size of the data to be read
 *
 * @retval < 0 error
 * @retval >= 0 number of bytes read from the UART module
 */
int hal_uart_read(uint8_t* buffer, uint16_t size);

/**
 * @brief Write to the UART module
 *
 * @param [out] buffer Buffer containing data to be written to the UART module
 * @param [in] size Size of the data to be written
 *
 * @retval < 0 error
 * @retval >= 0 number of bytes written to the UART module
 */
int hal_uart_write(uint8_t* buffer, uint16_t size);

#endif /* HAL_HAL_UART_H_ */
