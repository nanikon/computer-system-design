/*
 * uart_irq.h
 *
 *  Created on: Oct 4, 2023
 *      Author: natan
 */
#include "uart_irq.h"

uint8_t buffer_to_write[WRITE_BUFFER_SIZE] = {0}; // буфер на передачу
size_t start_write = 0; // номер символа с которого начинать передачу
size_t end_write = 0; // номер символа следом за последним символом, который надо передать

static void send_buffer_if_not_empty(UART_HandleTypeDef *huart) {
	if (end_write < start_write){
		// буфер закольцован, надо передать данные до его конца, и следующим вызовом с начала
		HAL_UART_Transmit_IT(huart6, buffer_to_write + start_write, WRITE_BUFFER_SIZE - start_write);
		start_write = 0;
	} else if (end_write > start_write) {
		// стандартная ситуация, просто загружаем данные в отправку
		HAL_UART_Transmit_IT(huart6, buffer_to_write + start_write, end_write - start_write);
	}
	// если равны то нет данных на отправку, завершаемся
}

void send(UART_HandleTypeDef *huart, uint8_t* buffer, size_t buf_size) {
	// добавить данные в буффер
	uint8_t state = HAL_UART_GetState(huart);
	if (buf_size > WRITE_BUFFER_SIZE - end_write) {
		size_t first_size = WRITE_BUFFER_SIZE - end_write;
		if (first_size > 0) {

		}
	} else {
		memcpy(buffer_to_write + end_write, buffer, buf_size);
		end_write += buf_size;
	}
	if (state != HAL_UART_STATE_BUSY_TX) {
		// свободно, начать передачу
		send_buffer_if_not_empty(huart);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
      // USART6 завершил отправку данных
	  send_buffer_if_not_empty(huart);
	  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
  }
}
