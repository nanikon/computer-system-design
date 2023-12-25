#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_LIGHT_DURATION 10
#define RED_COLOR 1
#define YELLOW_COLOR 0
#define GREEN_COLOR 1
#define MODE_SIZE 5
#define CUSTOM_MODE_SIZE 100

typedef struct {
  uint8_t color; // 0 для красного и зелёного, 1 для жёлтого
  uint8_t duration; // для свечения в такте 0..9
} Tick;

typedef struct {
  uint8_t color; // 0 для красного и зелёного, 1 для жёлтого
  uint8_t duration; // для свечения в такте 0..9
  uint8_t brightness;
  uint8_t softness; // 0 --  нет мягкого перехода, 1 -- есть
} Input_tick;

typedef struct {
  Tick *green;
  Tick *red_yellow;
  uint8_t len;
} Mode;

void fill_mode_array(Input_tick *array, uint8_t len, Mode *mode);

void zero_tick(Tick *tick) {
  tick->color = 1; // нам не важно какой цвет, важно, что нулевой
  tick->duration = 0;
}

void fill_tick(Tick *tick, uint8_t color, uint8_t duration) {
  tick->color = color;
  tick->duration = duration;
}

Tick *define_color(char color, uint8_t *new_color, Tick **other_buf,
                   Mode *mode) {
  Tick *buffer;
  switch (color) {
  case 'r':
    *new_color = RED_COLOR;
    buffer = mode->red_yellow;
    *other_buf = mode->green;
    break;
  case 'y':
    *new_color = YELLOW_COLOR;
    buffer = mode->red_yellow;
    *other_buf = mode->green;
    break;
  case 'g':
    *new_color = GREEN_COLOR;
    buffer = mode->green;
    *other_buf = mode->red_yellow;
    break;
  }
  return buffer;
}

void fill_tick_array(char color, uint8_t brightness, uint8_t duration,
                     int8_t softness, Mode *mode) {
  uint8_t new_color;
  Tick *other_buffer;
  Tick *buffer = define_color(color, &new_color, &other_buffer, mode);
  uint8_t n = duration * DEFAULT_LIGHT_DURATION; // количество тиков, которое
                                                 // будет занимать этот цвет
  int each = 1;
  int left = abs(softness - (n / 2));
  int left_v = 0;

  if (softness > 0) {
    left_v = 1;
    if (abs(softness / (n / 2)) > 1) {
      each = 1;
    }
  }

  if (softness < 0) {
    each = -each;
    left_v = -left_v;
  }

  printf("bright=%i, dur=%i, soft=%i, each=%i, letf=%i, letf_v=%i\n",
         brightness, duration, softness, each, left, left_v);

  int cur_bright = brightness - softness;
  for (int i = mode->len; i < n + mode->len; i++) {
    if (i < (n / 2)) {
      if (i < (n / 2) && abs(brightness - cur_bright) >= abs(each)) {
        cur_bright += each;
      }

      if (abs(brightness - cur_bright) >= abs(left_v) && left > 0) {
        cur_bright += left_v;
        left--;
      }
    }
    fill_tick(&buffer[i], new_color, cur_bright);
    zero_tick(&other_buffer[i]);
  }
  mode->len += n;
}

void fill_mode_array(Input_tick *array, uint8_t len, Mode *mode) {
  for (int i = 0; i < len; i++) {
    Input_tick cur = array[i];

    int8_t soft = 0;
    if (cur.softness == 1) {
      if (i != 0 &&
          array[i - 1].brightness != array[i].brightness) { //ситуация 0 1
        soft = array[i].brightness - array[i - 1].brightness;
      }
    }
    printf("=====TICK %i========\n", i);
    fill_tick_array(cur.color, cur.brightness, cur.duration, soft, mode);
    for (int i = 0; i < mode->len; i++) {
      printf("%i(g%i r%i%i) ", i, mode->green[i].duration,
             mode->red_yellow[i].duration, mode->red_yellow[i].color);
    }
    printf("\nlen%i\n", mode->len);
  }
}

int main() {
  Mode *mode = (Mode *)malloc(sizeof(Mode));
  mode->len = 0;
  mode->green = (Tick *)calloc(sizeof(Tick), CUSTOM_MODE_SIZE);
  mode->red_yellow = (Tick *)calloc(sizeof(Tick), CUSTOM_MODE_SIZE);
  Input_tick arr[2];
  arr[0] =
      (Input_tick){.color = 'r', .duration = 5, .brightness = 9, .softness = 1};
  arr[1] =
      (Input_tick){.color = 'g', .duration = 3, .brightness = 3, .softness = 0};
  fill_mode_array(arr, 2, mode);
}