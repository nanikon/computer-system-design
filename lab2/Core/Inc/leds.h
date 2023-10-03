/*
 * leds.h
 *
 *  Created on: Sep 22, 2023
 *      Author: natan
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

#include "stm32f4xx_hal.h"

void turn_on_yellow_led();
void turn_off_yellow_led();

void turn_on_red_led();
void turn_off_red_led();

void turn_on_green_led();
void turn_off_green_led();

#endif /* INC_LEDS_H_ */
