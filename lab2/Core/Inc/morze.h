/*
 * morze.h
 *
 *  Created on: 7 нояб. 2023 г.
 *      Author: senin
 */

#ifndef INC_MORZE_H_
#define INC_MORZE_H_

#include "string.h"
char* char_to_morze(char c);
int str_to_morze(char* buf_in, int ptr_in, int* buf_out, int ptr_out);
char morze_to_str(int b[], int p);

#endif /* INC_MORZE_H_ */
