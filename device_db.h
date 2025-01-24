/*
 * device_db.h
 *
 *  Created on: Jan 8, 2025
 *      Author: ROJ030
 */

#ifndef DEVICE_DB_H_
#define DEVICE_DB_H_

#include <stdint.h>

/**
 * @brief Add a device into the dabase
 *
 * @retval 0 Success
 * @retval -1 Overflow
 * @retval -2 Already inside database
 */
int device_db_add(uint8_t* address);

/**
 * @brief Check if address is inside the database or not
 *
 * @retval-10 not in database
 * @retval > 0 address of device in the database
 */
int device_db_is_inside(uint8_t* address);

#endif /* DEVICE_DB_H_ */
