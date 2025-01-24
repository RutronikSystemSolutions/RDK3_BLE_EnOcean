/******************************************************************************
* File Name:   main.c
*
* Description: This example demonstrates how to use the RDK3 in combination with
* a TextToSpeech adapter board to interact with PTM216B switches from EnOcean
*
* Related Document: See README.md
*
*
*  Created on: 2022-12-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Author: ROJ030, GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "hal/hal_i2c.h"

#include "host_main.h"
#include "battery_booster.h"

#include "epson_tts.h"
#include "device_db.h"

/**
 * Watchdog timer
 * Timeout of 4 seconds
 * If the software does not trigger the WDT during this timeout, then a restart happens
 */
#define WDT_TIMEOUT_MS	4000

static cyhal_timer_t sensor_timer;
static uint8_t timer_interrupt = 0;
static cyhal_wdt_t watchdog;
static uint8_t store_next = 0;

static void sensor_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
	(void) callback_arg;
	(void) event;

	timer_interrupt = 1;
}

static cy_rslt_t sensor_timer_init(void)
{
	const uint32_t timer_frequency_hz = 10000;
	const uint32_t isr_frequency_hz = 100;

	const uint8_t priority = 6;
	cyhal_timer_cfg_t configuration;
	cy_rslt_t result;

	configuration.compare_value = 0;
	configuration.period = (timer_frequency_hz / isr_frequency_hz);
	configuration.direction = CYHAL_TIMER_DIR_UP;
	configuration.is_compare = false;
	configuration.is_continuous = true;
	configuration.value = 0;

	result = cyhal_timer_init(&sensor_timer, NC, NULL);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_timer_configure(&sensor_timer, &configuration);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_timer_set_frequency(&sensor_timer, timer_frequency_hz);
	if (result != CY_RSLT_SUCCESS) return result;

	cyhal_timer_register_callback(&sensor_timer, sensor_timer_isr, NULL);

	cyhal_timer_enable_event(&sensor_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, priority, true);

	result = cyhal_timer_start(&sensor_timer);
	return result;
}

static void init_watchdog()
{
	cyhal_wdt_init(&watchdog, WDT_TIMEOUT_MS);
}

static void print_address(uint8_t* address)
{
	printf("0x%x:0x%x:0x%x:0x%x:0x%x:0x%x\r\n", address[0], address[1], address[2], address[3], address[4], address[5]);
}

void switch_listener(uint8_t* address, uint8_t type, uint16_t manufacturer_id, uint32_t counter, uint8_t switch_status)
{
	const uint8_t REQUESTED_TYPE = 0xFF;
	const uint16_t REQUESTED_MANUFACTURED_ID = 0x3da;

	// Only react to PTM216B switches
	if ((type == REQUESTED_TYPE) && (manufacturer_id == REQUESTED_MANUFACTURED_ID))
	{
//		printf("Type: %d \r\n", type);
//		printf("Manufacturer ID: %d \r\n", manufacturer_id);
//		printf("Counter: %lu \r\n", counter);
//		printf("Switch status: %d \r\n", switch_status);

		// Store the address to DB?
		if (store_next)
		{
			store_next = 0;
			printf("Store address\r\n");
			print_address(address);
			if (device_db_add(address) != 0)
			{
				printf("Error while adding address.\r\n");
			}
			cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);
		}

		// Only play by press
		if (switch_status & 1)
		{
			int db_index = device_db_is_inside(address);
			printf("Db index; %d\r\n", db_index);
			if (db_index >= 0)
			{
				if (db_index == 0)
					epson_tts_play_sound_async(0);
				else
					epson_tts_play_sound_async(1);
			}
		}

		cyhal_gpio_toggle(LED2);
	}
}

/**
 * @brief Interrupt handler used to detect that the USR_BTN has been pressed
 */
void btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    store_next = 1;
    cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_ON);
}

static void handle_error()
{
	cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_ON);
	cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_ON);
	cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_ON);
	for(;;){}
}

int main(void)
{
    cy_rslt_t result;
    int8_t res = 0;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
    	for(;;){}
    }

    __enable_irq(); /* Enable global interrupts. */

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
    	for(;;){}
    }

    printf("------------------------- \r\n");
    printf("Starting RDK3 BLE EnOcean v1.1 \r\n");
    printf("Enabling to scan EnOcean switches \r\n");
    printf("------------------------- \r\n");

    /* Initialize the User LEDs */
    result = cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("cyhal_gpio_init error\r\n");
    	for(;;){}
    }

    cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_ON);
    cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);

    // Initialize I2C master -> needed to control the battery booster
	res = hal_i2c_init();
	if (res != 0)
	{
		handle_error();
	}

    // Initialise timer
    res = sensor_timer_init();
    if (res != 0)
    {
    	printf("sensor_timer_init error: %d \r\n", res);
    	handle_error();
    }

    /*Charger control*/
    result = cyhal_gpio_init(CHR_DIS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("cyhal_gpio_init CHR_DIS error\r\n");
    	handle_error();
    }
    cyhal_gpio_write((cyhal_gpio_t)CHR_DIS, false); /*Charger ON*/

    /* Configure USER_BTN */
    cyhal_gpio_callback_data_t btn_data =
    {
    		.callback = btn_interrupt_handler,
    		.callback_arg = NULL,
    };

    result = cyhal_gpio_init(USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    if (result != CY_RSLT_SUCCESS)
	{
		printf("cyhal_gpio_init USER_BTN error\r\n");
		handle_error();
	}
    cyhal_gpio_register_callback(USER_BTN, &btn_data);
    cyhal_gpio_enable_event(USER_BTN, CYHAL_GPIO_IRQ_FALL, BTN_IRQ_PRIORITY, true);

    /* Li-ION/Li-Po battery Booster Initialisation */
    if(!batt_boost_ctrl_init())
    {
    	printf("Battery power supply failure.\r\n");
    	handle_error();
    }

    // Text to Speech
    if (epson_tts_init() != 0)
    {
    	printf("Cannot init EPSON Text To Speech.\r\n");
    	handle_error();
    }
    epson_tts_play_sound_async(0);


    Ble_Init();
    host_main_set_listener(switch_listener);

    init_watchdog();

    uint32_t counter = 0;

    for (;;)
    {
    	if (timer_interrupt)
    	{
    		timer_interrupt = 0;

    		if ((counter % 10) == 0)
			{
    			cyhal_gpio_toggle(LED1);
			}
    		counter++;
    	}

    	host_main_do();

    	epson_tts_do();

    	cyhal_wdt_kick(&watchdog);
    }
}

/* [] END OF FILE */
