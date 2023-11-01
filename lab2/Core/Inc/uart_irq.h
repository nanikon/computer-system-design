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

void send(uint8_t* buffer, size_t buf_size);
void receive(uint8_t* buffer, size_t buf_size);

#endif /* INC_UART_IRQ_H_ */
