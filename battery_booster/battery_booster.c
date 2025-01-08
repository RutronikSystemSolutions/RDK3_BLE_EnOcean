/*
 * battery_booster.c
 *
 *  Created on: 2023-12-13
 *      Author: Gintaras
 */

#include "dio59020.h"
#include "battery_booster.h"
#include "hal/hal_i2c.h"

dio_monitor_reg_status_t dio_m_status;
charge_stat_t charge_stat = CHRG_FAULT;
chrg_fault_t chrg_fault = CHRG_NORMAL;

_Bool batt_boost_ctrl_init(void)
{
	cy_rslt_t result;

	/*Charger BOOST ENABLE pin stays low by default*/
    result = cyhal_gpio_init(BOOST_EN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	return false;
    }

    /*I2C read write function for DIO59020 Control*/
    dio59020_init(hal_i2c_read_register, hal_i2c_write_register);

    /*Ping the charger*/
    if(dio_online())
    {
    	/*The charging currents depends on your battery used*/
    	dio_set_batt_current(CURR_VREF_101_8);

    	/*Check if a button is pressed*/
    	if(!cyhal_gpio_read(USER_BTN))
    	{
    		CyDelay(500);
    		/*Check it once more*/
    		if(!cyhal_gpio_read(USER_BTN))
    		{
    			/*The button is pressed*/
    			/*Unload the DIO59020 and turn on the +5V booster*/
    			CyDelay(10);
    			dio_booster_enable();

    			/*Load the DIO59020 now*/
    			CyDelay(100);
    			return true;
    		}
    		else /*Noise?*/
    		{
    			return true;
    		}
    	}
    	else
    	{
    		dio_booster_disable();
    		return true;
    	}
    }

	return false;
}
