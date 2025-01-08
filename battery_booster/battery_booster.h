/*
 * battery_booster.h
 *
 *  Created on: 2023-12-13
 *      Author: Gintaras
 */

#ifndef BATT_BOOST_H_
#define BATT_BOOST_H_

#include "cyhal.h"
#include "cybsp.h"

/*Priority for button interrupts*/
#define BTN_IRQ_PRIORITY		5

_Bool batt_boost_ctrl_init(void);

#endif /* BATT_BOOST_H_ */
