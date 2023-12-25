#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

uint8_t Check_Row(uint8_t  Nrow, I2C_HandleTypeDef hi2c1 );
HAL_StatusTypeDef Set_Keyboard(I2C_HandleTypeDef hi2c1);

#endif /* INC_KEYBOARD_H_ */
