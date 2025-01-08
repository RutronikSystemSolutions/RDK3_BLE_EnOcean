/*
 * advertisement_packet.h
 *
 *  Created on: 24 Nov 2023
 *      Author: jorda
 */

#ifndef ADVERTISEMENT_PACKET_H_
#define ADVERTISEMENT_PACKET_H_

#include <stdint.h>

typedef struct
{
	uint8_t type;
	uint16_t manufacturer_id;
	uint32_t counter;
	uint8_t switch_status;
} enocean_packet_t;

int advertisement_packet_extract_name(uint8_t* packet, uint16_t packet_len, uint8_t* name, uint16_t name_len);

int advertisement_packet_extract_enocean(uint8_t* packet, uint16_t packet_len, enocean_packet_t* enocean_packet);

#endif /* ADVERTISEMENT_PACKET_H_ */
