/*
 * host_main.h
 *
 *  Created on: 24 Mar 2023
 *      Author: jorda
 */

#ifndef HOST_MAIN_H_
#define HOST_MAIN_H_

#include <stdint.h>

#define BLE_ADDRESS_LEN 6

typedef void (*host_main_switch_listener_func_t)(uint8_t* address, uint8_t type, uint16_t manufacturer_id, uint32_t counter, uint8_t switch_status);

void host_main_set_listener(host_main_switch_listener_func_t listener);

#define BLEMAX_MTU_SIZE	512

#define BLE_CMD_PARAM_MAX_SIZE (BLEMAX_MTU_SIZE - 1) // -1 because first by is the command type
#define BLE_ACK_MAX_SIZE 32

#define BLE_MODE_CONFIGURATION	1
#define BLE_MODE_PUSH_DATA 		2

typedef struct
{
	uint8_t command;
	uint16_t len;
	uint8_t parameters[BLE_CMD_PARAM_MAX_SIZE];
} ble_cmt_t;


typedef struct
{
	uint8_t notification_enabled;			/**< Store if notification on the characteristic are enabled or not. The app must activate them */

	uint8_t cmd_to_process;					/**< Store if a command is ready to be processed or not */
	ble_cmt_t cmd;							/**< Store the command to be processed (once available) */

	uint8_t mode;							/**< Store the actual configuration mode */
} host_main_t;

void Ble_Init();

int host_main_do();

int host_main_scan_for_peripherals();
int host_main_stop_scan();

#endif /* HOST_MAIN_H_ */
