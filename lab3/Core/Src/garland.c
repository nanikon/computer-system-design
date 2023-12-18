#include "garland.h"

/*
формат ввода:
цвет (r/y/g),
яркость (от 1 до 9),
длительность (от 1 до 9),
наличие плавного перехода (+/-).
Если неизвестный символ, то типо начинаем с начала этот терм.
Пример: r79+y29-*/

void zero_tick(Tick tick){
  tick.color = 1; // нам не важно какой цвет, важно, что нулевой
  tick.duration = 0;
}

void fill_tick(Tick tick, uint8_t color, uint8_t duration){
  tick.color = color;
  tick.duration = duration;
}

Tick* define_color(char color, uint8_t* new_color, Tick** other_buf, Tick* green, Tick* red_yellow){
  Tick* buffer;
  switch (color){
    case 'r':
      *new_color = 1;
      buffer = red_yellow;
      *other_buf = green;
      break;
    case 'y':
      *new_color = 0;
      buffer = red_yellow;
      *other_buf = green;
      break;
    case 'g':
      *new_color = 1;
      buffer = green;
      *other_buf = red_yellow;
      break;
  }
  return buffer;
}

void fill_tick_array(char color, uint8_t brightness, uint8_t duration, int8_t softness, Tick* green, Tick* red_yellow, uint32_t* writing_ptr){
  uint8_t new_color;
  Tick* other_buffer;
  Tick* buffer = define_color(color, &new_color, &other_buffer, green, red_yellow);
  uint8_t n = duration*DEFAULT_LIGHT_DURATION; // количество тиков, которое будет занимать этот цвет
  int each = 1;
  int left = abs(softness - (n/2));
  int left_v = 0;

  if (softness > 0){
    left_v = 1;
    if (abs(softness / (n/2)) > 1){
        each = 1;
    }
  }

  if (softness < 0){
    each = -each;
    left_v = -left_v;
  }

  int cur_bright = brightness - softness;
  for(int i = 0; i < n; i++) {
    if(i < (n/2)){
        if(i < (n/2) && abs(brightness - cur_bright) >= abs(each)) {
          cur_bright += each;
        }

        if(abs(brightness - cur_bright) >= abs(left_v) && left > 0) {
          cur_bright += left_v;
          left--;
        }
    }
    fill_tick(buffer[i], new_color, cur_bright);
    zero_tick(other_buffer[i]);
  }
  writing_ptr += n;
}

void fill_mode_array(Input_tick* array, uint8_t len, Tick* green, Tick* red_yellow, uint32_t* writing_ptr){
  for (int i = 0; i < len; i++){
    Input_tick cur = array[i];

    int8_t soft = 0;
    if (cur.softness == 1){
      if (i != 0 && array[i-1].brightness != array[i].brightness){ //ситуация 0 1
        soft = array[i].brightness - array[i-1].brightness;
      }
    }
    fill_tick_array(cur.color, cur.brightness, cur.duration, soft, green, red_yellow, writing_ptr);
  }
}

void play_green(uint8_t tick, Tick* green, uint32_t current_write_ptr){
  Tick cur = green[current_write_ptr];
  if (tick < cur.duration)  turn_on_green_led();
  else  turn_off_green_led();
}

void play_red_yellow(uint8_t tick, Tick* red_yellow, uint32_t current_write_ptr){
  Tick cur = red_yellow[current_write_ptr];
  if (cur.color == RED_COLOR){
    if (tick < cur.duration)  turn_on_red_led();
    else  turn_off_red_led();
  }else{
    if (tick < cur.duration)  turn_on_yellow_led();
    else  turn_off_yellow_led();
  }
}

void play_new_mode(Mode* mode, Tick* green, Tick* red_yellow, uint32_t* writing_ptr, uint32_t* current_write_ptr){
    memcpy(green, mode->green, MODE_SIZE);
    memcpy(red_yellow, mode->red_yellow, MODE_SIZE);
    *writing_ptr = MODE_SIZE;
    *current_write_ptr = 0;
}

void init_mode1(Mode* mode1){
    mode1->green[0] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode1->red_yellow[0] = (Tick){.color = RED_COLOR, .duration=9};
    mode1->green[1] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode1->red_yellow[1] = (Tick){.color = YELLOW_COLOR, .duration=9};
    mode1->green[2] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode1->red_yellow[2] = (Tick){.color = RED_COLOR, .duration=9};
    mode1->green[3] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode1->red_yellow[3] = (Tick){.color = YELLOW_COLOR, .duration=9};
    mode1->green[4] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode1->red_yellow[4] = (Tick){.color = RED_COLOR, .duration=9};
}

void init_mode2(Mode* mode2){
    mode2->green[0] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode2->red_yellow[0] = (Tick){.color = RED_COLOR, .duration=0};
    mode2->green[1] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode2->red_yellow[1] = (Tick){.color = RED_COLOR, .duration=0};
    mode2->green[2] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode2->red_yellow[2] = (Tick){.color = RED_COLOR, .duration=0};
    mode2->green[3] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode2->red_yellow[3] = (Tick){.color = RED_COLOR, .duration=0};
    mode2->green[4] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode2->red_yellow[4] = (Tick){.color = RED_COLOR, .duration=0};
}

void init_mode3(Mode* mode3){
    mode3->green[0] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode3->red_yellow[0] = (Tick){.color = RED_COLOR, .duration=9};
    mode3->green[1] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode3->red_yellow[1] = (Tick){.color = YELLOW_COLOR, .duration=0};
    mode3->green[2] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode3->red_yellow[2] = (Tick){.color = YELLOW_COLOR, .duration=9};
    mode3->green[3] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode3->red_yellow[3] = (Tick){.color = RED_COLOR, .duration=0};
    mode3->green[4] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode3->red_yellow[4] = (Tick){.color = RED_COLOR, .duration=9};
}

void init_mode4(Mode* mode4){
    mode4->green[0] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode4->red_yellow[0] = (Tick){.color = RED_COLOR, .duration=9};
    mode4->green[1] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode4->red_yellow[1] = (Tick){.color = RED_COLOR, .duration=0};
    mode4->green[2] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode4->red_yellow[2] = (Tick){.color = RED_COLOR, .duration=9};
    mode4->green[3] = (Tick){.color = GREEN_COLOR, .duration=0};
    mode4->red_yellow[3] = (Tick){.color = RED_COLOR, .duration=0};
    mode4->green[4] = (Tick){.color = GREEN_COLOR, .duration=9};
    mode4->red_yellow[4] = (Tick){.color = RED_COLOR, .duration=9};
}

void init_modes(Mode* modes){
    init_mode1(&modes[0]);
    init_mode2(&modes[1]);
    init_mode3(&modes[2]);
    init_mode4(&modes[3]);
}
