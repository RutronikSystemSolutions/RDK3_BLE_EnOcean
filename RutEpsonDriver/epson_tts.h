/*
 * epson_tts.h
 *
 *  Created on: Jan 8, 2025
 *      Author: ROJ030
 */

#ifndef RUTEPSONDRIVER_EPSON_TTS_H_
#define RUTEPSONDRIVER_EPSON_TTS_H_

#include <stdint.h>

/**
 * @brief Initialize the Text To Speech board (GPIOs, SPI)
 *
 * @retval 0 on success
 */
int epson_tts_init();

/**
 * @brief Play a sound (stored previously inside the memory)
 *
 * @retval 0 on success
 */
int epson_tts_play_sound_async(uint8_t index);

/**
 * @brief Cyclic call (enable to mute/unmute when sound is over)
 */
void epson_tts_do();


#endif /* RUTEPSONDRIVER_EPSON_TTS_H_ */
