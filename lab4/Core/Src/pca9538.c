#include "pca9538.h"

HAL_StatusTypeDef PCA9538_Read_Register(uint16_t addr, pca9538_regs_t reg, uint8_t* buf, I2C_HandleTypeDef hi2c1) {
	return HAL_I2C_Mem_Read(&hi2c1, addr | 1, reg, 1, buf, 1, 100);
}


HAL_StatusTypeDef PCA9538_Write_Register(uint16_t addr, pca9538_regs_t reg, uint8_t* buf, I2C_HandleTypeDef hi2c1) {
	return HAL_I2C_Mem_Write(&hi2c1, addr & 0xFFFE, reg, 1, buf, 1, 100);
}


HAL_StatusTypeDef PCA9538_Read_Inputs(uint16_t addr, uint8_t* buf, I2C_HandleTypeDef hi2c1) {
	return PCA9538_Read_Register(addr, INPUT_PORT, buf, hi2c1);
}

