/*
 * advertisement_packet.c
 *
 *  Created on: 24 Nov 2023
 *      Author: jorda
 */

#include "advertisement_packet.h"

static const uint8_t complete_local_name = 0x09;

int advertisement_packet_extract_name(uint8_t* packet, uint16_t packet_len, uint8_t* name, uint16_t name_len)
{
	// Composed of chunks
	// Length
	// Type (flags, complete local name, complete list of 16-bit UUIDs available, ...)
	// Data (length - 1)
	uint16_t index = 0;
	for(;;)
	{
		uint8_t chunk_len = packet[index];
		uint8_t chunk_type = packet[index+1];
		uint16_t remaining_space = packet_len - index;

		if (chunk_len == 0) return -2;
		if (chunk_len > remaining_space) return -3;

		if (chunk_type == complete_local_name)
		{
			// Found local name, extract it
			// (chunk_len - 1) because chunk_len also contains the type

			if ((chunk_len - 1) > name_len) return -4;

			uint16_t i = 0;
			for(i = 0; i  < (chunk_len - 1); ++i)
			{
				// index +2 because: [0] is length and [1] is type
				name[i] = packet[index + i + 2];
			}
			name[i] = '\0';
			return (i+1);
		}

		// Increase index
		index += chunk_len + 1;
		if (index >= packet_len) break;
	}

	return -1;
}

int advertisement_packet_extract_enocean(uint8_t* packet, uint16_t packet_len, enocean_packet_t* enocean_packet)
{
	const uint16_t min_size = 0x0C;
	const uint16_t max_size = 0x10;

	if (packet_len < min_size) return -1;
	if (packet_len > max_size) return -2;


	uint16_t manufacturer_id = *((uint16_t*)&packet[2]);
	uint32_t counter = *((uint32_t*)&packet[4]);

	// Fill packet
	enocean_packet->type = packet[1];
	enocean_packet->manufacturer_id = manufacturer_id;
	enocean_packet->counter = counter;
	enocean_packet->switch_status = packet[8];

	return 0;
}


