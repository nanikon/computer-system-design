#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "leds.h"

#define DEFAULT_LIGHT_DURATION 10
#define RED_COLOR 1
#define YELLOW_COLOR 0
#define GREEN_COLOR 1
#define MODE_SIZE 5

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

typedef struct{
    Tick green[MODE_SIZE];
    Tick red_yellow[MODE_SIZE];
} Mode;

void init_modes(Mode* modes);
void play_new_mode(Mode* mode, Tick* green, Tick* red_yellow, uint32_t* writing_ptr, uint32_t* current_write_ptr);

void fill_mode_array(Input_tick* array, uint8_t len, Tick* green, Tick* red_yellow, uint32_t* writing_ptr);
void play_green(uint8_t tick, Tick* green, uint32_t current_write_ptr);
void play_red_yellow(uint8_t tick, Tick* red_yellow, uint32_t current_write_ptr);
