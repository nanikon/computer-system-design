/*
 * garland.h
 *
 *  Created on: 18 дек. 2023 г.
 *      Author: natan
 */

#ifndef INC_GARLAND_H_
#define INC_GARLAND_H_

#include <inttypes.h>
#include <math.h>
#include "leds.h"
#define DEFAULT_LIGHT_DURATION 10
#define RED_COLOR 1

typedef struct{
  uint8_t color; // 0 для красного и зелёного, 1 для жёлтого
  uint8_t duration; // для свечения в такте 0..9
}Tick;

typedef struct{
  uint8_t color; // 0 для красного и зелёного, 1 для жёлтого
  uint8_t duration; // для свечения в такте 0..9
  uint8_t brightness;
  uint8_t softness; // 0 --  нет мягкого перехода, 1 -- есть
}Input_tick;

void fill_mode_array(Input_tick* array, uint8_t len, Tick* green, Tick* red_yellow, uint32_t writing_ptr);
void play_green(uint8_t tick, Tick* green, uint32_t current_write_ptr);
void play_red_yellow(uint8_t tick, Tick* red_yellow, uint32_t current_write_ptr);

#endif /* INC_GARLAND_H_ */
