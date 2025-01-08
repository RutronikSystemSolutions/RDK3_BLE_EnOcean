/*
 * hal_sleep.h
 *
 *  Created on: 24 Mar 2023
 *      Author: jorda
 */

#ifndef HAL_HAL_SLEEP_H_
#define HAL_HAL_SLEEP_H_

#include <stdint.h>

void hal_sleep(uint32_t ms);

void hal_sleep_us(uint16_t us);

#endif /* HAL_HAL_SLEEP_H_ */
