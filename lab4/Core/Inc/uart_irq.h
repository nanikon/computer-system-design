/*
 * uart_irq.h
 *
 *  Created on: Dec 25, 2023
 *      Author: natan
 */

#ifndef INC_UART_IRQ_H_
#define INC_UART_IRQ_H_

#include "stm32f4xx_hal.h"
#include <string.h>

#define WRITE_BUFFER_SIZE 512

void send_uart(UART_HandleTypeDef *huart, uint8_t* buffer);

#endif /* INC_UART_IRQ_H_ */
