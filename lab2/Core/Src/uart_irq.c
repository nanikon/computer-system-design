/*
 * uart_irq.h
 *
 *  Created on: Oct 4, 2023
 *      Author: natan
 */
#include "uart_irq.h"
#include "string.h"

char buffer_to_write[WRITE_BUFFER_SIZE] = {0}; // буфер на передачу
size_t start_write = 0; // номер символа с которого начинать передачу
size_t end_write = 0; // номер символа следом за последним символом, который надо передать

void send_buffer_if_not_empty_IT(UART_HandleTypeDef *huart){
	
	if (end_write < start_write){
		// буфер закольцован, надо передать данные до его конца, и следующим вызовом с начала
		HAL_UART_Transmit_IT(huart, (uint8_t*)(buffer_to_write + start_write), WRITE_BUFFER_SIZE - start_write);
		start_write = 0;
	} else if (end_write > start_write) {
		// стандартная ситуация, просто загружаем данные в отправку
		HAL_UART_Transmit_IT(huart, (uint8_t*)(buffer_to_write + start_write), end_write - start_write);
		// если равны то нет данных на отправку, завершаемся
	}
	
}

void send_buffer_if_not_empty(UART_HandleTypeDef *huart){
	char state = HAL_UART_GetState(huart);
	if (state != HAL_UART_STATE_BUSY_TX) {
		if (end_write < start_write){
			// буфер закольцован, надо передать данные до его конца, и следующим вызовом с начала
			HAL_UART_Transmit(huart, (uint8_t*)(buffer_to_write + start_write), WRITE_BUFFER_SIZE - start_write, 100);
			start_write = 0;
		} else if (end_write > start_write) {
			// стандартная ситуация, просто загружаем данные в отправку
			HAL_UART_Transmit(huart, (uint8_t*)(buffer_to_write + start_write), end_write - start_write, 100);
		}
	}
}

void send_uart(UART_HandleTypeDef *huart, char* buffer, size_t buf_size, int has_irq) {
	// добавить данные в буффер
	char state = HAL_UART_GetState(huart);
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
		if (has_irq == 1) send_buffer_if_not_empty_IT(huart);
	}
}

void recive_uart(UART_HandleTypeDef *huart, char* buffer, size_t size, int has_irq){ //проверить, что таким образом я считала всё
	if (has_irq == 1) {
		HAL_UART_Receive_IT(huart, (uint8_t*)buffer, size);
	}else{
		HAL_StatusTypeDef stat = HAL_UART_Receive(huart, (uint8_t*)buffer, size, 0);
		switch (stat) {
			case HAL_OK: {
				send_uart(huart, buffer, size, has_irq);
				return;
			}
			case HAL_ERROR:
			case HAL_BUSY:
			case HAL_TIMEOUT:
			break;
		}
	}
}