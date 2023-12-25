/*
 * uart.c
 *
 *  Created on: Dec 25, 2023
 *      Author: natan
 */
#include "uart_irq.h"

uint8_t buffer_to_write[WRITE_BUFFER_SIZE] = {}; // буфер на передачу
size_t start_write = 0; // номер символа с которого начинать передачу
size_t end_write = 0;

void send_buffer_if_not_empty_IT(UART_HandleTypeDef *huart){
	if (start_write != end_write) {
		HAL_UART_Transmit_IT(huart, (uint8_t*) (buffer_to_write + start_write), 1);
		start_write++;
		if (start_write == WRITE_BUFFER_SIZE) {
			start_write = 0;
			if (end_write == WRITE_BUFFER_SIZE) {
				end_write = 0;
			}
		}
	}
}

void send_uart(UART_HandleTypeDef *huart, uint8_t* buffer) {
	// добавить данные в буффер
	char state = HAL_UART_GetState(huart);
	size_t buf_size = strlen((const char*)buffer);
	if (buf_size > WRITE_BUFFER_SIZE - end_write) {
		size_t first_size = WRITE_BUFFER_SIZE - end_write;
		if (first_size > 0) {
			memcpy(buffer_to_write + end_write, buffer, first_size);
		}
		memcpy(buffer_to_write, buffer, buf_size - first_size);
		end_write = buf_size - first_size;
	} else {
		memcpy(buffer_to_write + end_write, buffer, buf_size);
		end_write += buf_size;
	}

	if (state != HAL_UART_STATE_BUSY_TX) {
		// свободно, начать передачу
		send_buffer_if_not_empty_IT(huart);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART6)
  {
      // USART6 завершил отправку данных
	  send_buffer_if_not_empty_IT(huart);
  }
}
