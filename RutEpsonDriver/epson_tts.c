/*
 * epson_tts.c
 *
 *  Created on: Jan 8, 2025
 *      Author: ROJ030
 */

#include "epson_tts.h"

#include "spi_api.h"

#include <stdio.h>

const uint8_t PLAYING_SOUND = 1;
const uint8_t IDDLE = 0;

static uint8_t sound_status = IDDLE;

int epson_tts_init()
{
	// Initialize Epson ASIC
	cy_rslt_t result = EPSON_Initialize();
	if(result != CY_RSLT_SUCCESS)
	{
		return -1;
	}

	// Epson ASIC Hard-Reset Procedure
	GPIO_S1V30340_Reset(1);
	CyDelay(100);
	GPIO_S1V30340_Reset(0);
	CyDelay(100);
	GPIO_S1V30340_Reset(1);
	CyDelay(500);

	// Set Epson ASIC mute signal(MUTE) to High(disable)
	GPIO_ControlMute(0);

	// Set Epson ASIC standby signal(STBYEXIT) to Low(deassert)
	GPIO_ControlStandby(0);
	CyDelay(100);

	// Configure Epson ASIC
	int retval = S1V30340_Initialize_Audio_Config();
	while(retval != 0)
	{
		printf("S1V30340 SPI Initialization failed. Trying again \n\r");
		CyDelay(1000);
		retval = S1V30340_Initialize_Audio_Config();
	}
	printf("S1V30340 SPI Initialization succeeded. \n\r");

	return 0;
}

int epson_tts_play_sound_async(uint8_t index)
{
	if (sound_status == PLAYING_SOUND) return -1;

	GPIO_ControlMute(1); /*Mute - OFF*/
	S1V30340_Play_Specific_Audio(index);

	sound_status = PLAYING_SOUND;

	return 0;
}

void epson_tts_do()
{
	if (sound_status == PLAYING_SOUND)
	{
		if (S1V30340_Is_Termination_Message_Received() == 1)
		{
			GPIO_ControlMute(0); // Mute - ON
			sound_status = IDDLE;
		}
	}
}

