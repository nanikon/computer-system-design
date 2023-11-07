/*
 * uart_irq.h
 *
 *  Created on: Oct 4, 2023
 *      Author: natan
 */

#ifndef INC_UART_IRQ_H_
#define INC_UART_IRQ_H_

#include "stm32f4xx_hal.h"

#define WRITE_BUFFER_SIZE 8192

void send_buffer_if_not_empty(UART_HandleTypeDef *huart);
void send_buffer_if_not_empty_IT(UART_HandleTypeDef *huart);
void send_uart(UART_HandleTypeDef *huart, char* buffer, size_t buf_size, int has_irq);
void recive_uart(UART_HandleTypeDef *huart, char* buffer, size_t size, int has_irq);

#endif /* INC_UART_IRQ_H_ */
