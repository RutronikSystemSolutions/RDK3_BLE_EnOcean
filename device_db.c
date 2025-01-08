/*
 * device_db.c
 *
 *  Created on: Jan 8, 2025
 *      Author: ROJ030
 */

#include "device_db.h"

#include "host_main.h"

#define DEVICE_DB_MAX_COUNT 4

typedef struct
{
	uint8_t addr[BLE_ADDRESS_LEN];
} device_addr_t;

static device_addr_t db_devices[DEVICE_DB_MAX_COUNT];
static uint16_t db_count;

int device_db_add(uint8_t* address)
{
	if (db_count >= DEVICE_DB_MAX_COUNT) return -1;
	if (device_db_is_inside(address)) return -2;
	// Copy address
	for(uint16_t i = 0; i < BLE_ADDRESS_LEN; ++i)
		db_devices[db_count].addr[i] = address[i];
	// Update count
	db_count++;
	return 0;
}

static int is_same_addr(uint8_t* add0, uint8_t* add1)
{
	const uint8_t len = BLE_ADDRESS_LEN;
	for(uint8_t i = 0; i < len; ++i)
	{
		if (add0[i] != add1[i]) return 0;
	}
	return 1;
}

int device_db_is_inside(uint8_t* address)
{
	for(int device_idx = 0; device_idx < db_count; ++device_idx)
	{
		if (is_same_addr(address, db_devices[device_idx].addr))
			return 1;
	}
	return 0;
}


