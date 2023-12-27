//#include "main.h"
#include "pca9538.h"
#include "kb.h"
#include "uart_irq.h"

#define KBRD_RD_ADDR 0xE3
#define KBRD_WR_ADDR 0xE2
#define ROW1 0xFE //1111 1110
#define ROW2 0xFD //1111 1101
#define ROW3 0xFB //1111 1011
#define ROW4 0xF7 //1111 0111

HAL_StatusTypeDef Set_Keyboard(I2C_HandleTypeDef hi2c1) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buf;

	buf=0x70;
	ret = PCA9538_Write_Register(KBRD_WR_ADDR, CONFIG, &buf, hi2c1);
	if( ret != HAL_OK ) {
		//UART_Transmit("Error write config\n");
		goto exit;
	}

	buf = 0;
	ret = PCA9538_Write_Register(KBRD_WR_ADDR, OUTPUT_PORT, &buf, hi2c1);
	if( ret != HAL_OK ) {
		//UART_Transmit("Error write output\n");
	}

exit:
	return ret;
}

uint8_t Check_Row( uint8_t  Nrow, I2C_HandleTypeDef hi2c1 ) {
	uint8_t Nkey = 0x00;
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buf = 0;
	uint8_t kbd_in;

	ret = Set_Keyboard(hi2c1);

	buf = Nrow;

	ret = PCA9538_Write_Register(KBRD_WR_ADDR, OUTPUT_PORT, &buf, hi2c1);

	ret = PCA9538_Write_Register(KBRD_WR_ADDR, CONFIG, &buf, hi2c1);

	buf = 0;
	ret = PCA9538_Read_Inputs(KBRD_RD_ADDR, &buf, hi2c1);

	kbd_in = buf & 0x70;
	Nkey = kbd_in;
	if( kbd_in != 0x70) {
		if( !(kbd_in & 0x10) ) {
			Nkey = 0x04;
		} else if( !(kbd_in & 0x20) ) {
			Nkey = 0x02;
		} else if( !(kbd_in & 0x40) ) {
			Nkey = 0x01;
		} else {
			Nkey = 0x00;
		}
	}
	else Nkey = 0x00;

	return Nkey;
}

