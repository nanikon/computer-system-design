/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIDDLE_BUFFER_SIZE 100
#define INPUT_TICK_BUFFER_SIZE 20
#define TICK_BUFF_SIZE 200
#define MAX_DURATION 10
#define MODES_COUNT 5
#define LONG_PERIOD_CT 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
char middle_buffer[MIDDLE_BUFFER_SIZE] = {0};
uint8_t output = 1;      // сейчас читаем или выводим
Tick green[TICK_BUFF_SIZE] = {};
Tick red_yellow[TICK_BUFF_SIZE] = {};
uint8_t tick = 0;
uint32_t writing_ptr;
uint32_t current_write_ptr;
Mode modes[MODES_COUNT] = {0};

uint8_t mode = 1;
uint32_t min_scaler = 1;
uint32_t max_scaler = 100;
uint32_t scaler_speed = 10;
uint32_t new_scaler_speed;

Input_tick tick_buffer[INPUT_TICK_BUFFER_SIZE] = {};
Input_tick cur_read_tick = {0};
uint8_t tick_ptr = 0;
uint8_t tick_len = 0;

uint64_t count_tick = 0;

uint8_t is_pressed_but = 0;
uint8_t noisy_but = 0;

uint8_t is_pressed_key = 0;
uint8_t noisy_key = 0;

uint8_t kb_testing = 1; // режим тестирование клавиатуры - 1
                        // режим работы с гирлядой - 0

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)&htim1);
  init_modes(modes);
  play_new_mode(&modes[0], green, red_yellow, &writing_ptr, &current_write_ptr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint8_t p_key = get_pressed_key();
      if (p_key != 0 && noisy_key == 0) {
      	noisy_key = p_key;
      } else if (p_key != 0 && noisy_key == p_key) {
      	if (is_pressed_key == 0) {
      		is_pressed_key = 1;
      		handler_pressed_key(p_key);
      	}
      } else if (noisy_key != 0 && p_key == 0) {
      	noisy_key = 0;
      } else if (noisy_key == 0 && p_key == 0) {
      	is_pressed_key = 0;
      }
      HAL_Delay(15);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 30-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BUT_Pin */
  GPIO_InitStruct.Pin = BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void send_user_mode_param() {
  if (tick_len == 0 && tick_ptr == 0) {
    send_uart(&huart6, "Not found user configuration!\r");
  } else {
    for (int i = 0; i < tick_len; i++) {
      bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
      sprintf(middle_buffer,
              "\rcolor: %c, bright: %d, duration %d, softness %d\r",
              tick_buffer[i].color, tick_buffer[i].brightness,
              tick_buffer[i].duration, tick_buffer[i].softness);
      send_uart(&huart6, middle_buffer);
    }
    if (tick_ptr > 0) {
      bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
      sprintf(middle_buffer, "\rcolor: %c", cur_read_tick.color);
      send_uart(&huart6, middle_buffer);
      if (tick_ptr > 1) {
        bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
        sprintf(middle_buffer, ", bright: %d",
                cur_read_tick.brightness);
        send_uart(&huart6, middle_buffer);
        if (tick_ptr > 2) {
          bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
          printf(middle_buffer, ", duration: %d",
                 cur_read_tick.duration);
          send_uart(&huart6, middle_buffer);
        }
      }
      send_uart(&huart6, "\r");
    }
  }
}

void fill_softnes_and_save_input_tick(uint8_t softness) {
  tick_ptr = 0;
  cur_read_tick.softness = softness;
  memcpy(tick_buffer + tick_len, &cur_read_tick, sizeof(Input_tick));
  tick_len++;
}

void handler_input(uint8_t pressed_key) {
  if (output == 1) {
    switch (pressed_key) {
    case 1:
      // to next mode
      if (mode == 5) {
        mode = 1;
      } else {
        mode++;
      }
      play_new_mode(&modes[mode - 1], green, red_yellow, &writing_ptr,
                    &current_write_ptr);
      break;
    case 2:
      // to previos mode
      if (mode == 1) {
        mode = 5;
      } else {
        mode--;
      }
      play_new_mode(&modes[mode - 1], green, red_yellow, &writing_ptr,
                    &current_write_ptr);
      break;
    case 3:
      // faster
      new_scaler_speed = scaler_speed - round((double)scaler_speed / 10);
      if (new_scaler_speed == scaler_speed) {
        new_scaler_speed--;
      }
      if (new_scaler_speed > min_scaler) {
        scaler_speed = new_scaler_speed;
      }
      break;
    case 4:
      // slower
      new_scaler_speed = scaler_speed + round((double)scaler_speed / 10);
      if (new_scaler_speed == scaler_speed) {
        new_scaler_speed++;
      }
      if (new_scaler_speed < max_scaler) {
        scaler_speed = new_scaler_speed;
      }
      break;
    case 5:
      // send current user mode
      send_user_mode_param();
      break;
    case 12:
      // ввод в меню настройки
      output = 0;
      tick_len = 0;
      break;
    default:
      break;
    }
  } else {
    if (pressed_key == 12) {
      if (tick_len != 0) {
        fill_mode_array(tick_buffer, tick_len, &modes[MODES_COUNT - 1]);
        tick_ptr = 0;
      }
      output = 1;
    } else {
      switch (tick_ptr) {
      case 0:
        if (tick_len == TICK_BUFF_SIZE) {
          bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
          send_uart(&huart6, "user mode buffer is full!");
        } else {
        	if (pressed_key < 4) {
        		tick_ptr++;
        		if (pressed_key == 1) {
        			cur_read_tick.color = 'g';
        		} else if (pressed_key == 2) {
        			cur_read_tick.color = 'y';
        		} else if (pressed_key == 3) {
        		    cur_read_tick.color = 'r';
        		}
        	}
        }
        break;
      case 1:
        if (pressed_key >= 1 && pressed_key <= 9) {
          tick_ptr++;
          cur_read_tick.brightness = pressed_key;
        }
        break;
      case 2:
        if (pressed_key >= 1 && pressed_key <= 9) {
          tick_ptr++;
          cur_read_tick.duration = pressed_key;
        }
        break;
      case 3:
        if (pressed_key == 10) {
          fill_softnes_and_save_input_tick(1);
        } else if (pressed_key == 11) {
          fill_softnes_and_save_input_tick(0);
        }
        break;
      default:
        tick_ptr = 0;
        break;
      }
    }
  }
}

uint8_t get_pressed_key() {
	uint8_t count_pressed_key = 0;
	uint8_t rows[4] = { 0xF7, 0x7B, 0x3D, 0x1E }, checkedRow;
	uint8_t result = 0;
	for (int i = 0; i < 4; i++) {
		checkedRow = Check_Row(rows[i], hi2c1);
		switch (i) {
		case 0:
			switch (checkedRow) {
			case 0x0:
				break;
			case 0x01:
				result = 1;
				count_pressed_key++;
				break;
			case 0x02:
				result = 2;
				count_pressed_key++;
				break;
			case 0x04:
				result = 3;
				count_pressed_key++;
				break;
			default:
				count_pressed_key += 2;
			}
			break;
		case 1:
			switch (checkedRow) {
			case 0x0:
				break;
			case 0x01:
				result = 4;
				count_pressed_key++;
				break;
			case 0x02:
				result = 5;
				count_pressed_key++;
				break;
			case 0x04:
				result = 6;
				count_pressed_key++;
				break;
			default:
				count_pressed_key += 2;
			}
			break;
		case 2:
			switch (checkedRow) {
			case 0x0:
				break;
			case 0x01:
				result = 7;
				count_pressed_key++;
				break;
			case 0x02:
				result = 8;
				count_pressed_key++;
				break;
			case 0x04:
				result = 9;
				count_pressed_key++;
				break;
			default:
				count_pressed_key += 2;
			}
			break;
		case 3:
			switch (checkedRow) {
			case 0x0:
				break;
			case 0x01:
				result = 10;
				count_pressed_key++;
				break;
			case 0x02:
				result = 11;
				count_pressed_key++;
				break;
			case 0x04:
				result = 12;
				count_pressed_key++;
				break;
			default:
				count_pressed_key += 2;
			}
			break;
		}
	}
	if (count_pressed_key > 1) {
		result = 0;
	}
	return result;
}

void handler_pressed_key(uint8_t pressed_key) {
	if (kb_testing == 0) {
	// режим практической работы
	   handler_input(pressed_key);
	   if (output == 1) {
	     // режим проигрывания - выводим номер мелодии и скорость
	     bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
	     sprintf(middle_buffer, "mode: %d, speed: %ld", mode,
	        		        	                  scaler_speed);
	     send_uart(&huart6, middle_buffer);
	    } else {
	       // режим пользовательской конфигурации - выводим ту которая есть
	       // сейчас send current user configuration
	       send_user_mode_param();
	    }
	} else {
		// режим тестирования
		bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
		sprintf(middle_buffer, "%d", pressed_key);
		send_uart(&huart6, middle_buffer);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
	  tick++;
  if (tick % 5 == 0) { // логика работы с дребезгом и кнопками
	int but = HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin);
	but = !but;

	 if (but == 1 && noisy_but == 0) {
	    noisy_but = 1;
	 } else if (but == 1 && noisy_but == 1) {
	            if (is_pressed_but == 0) {
	          	  is_pressed_but = 1;
	          	  kb_testing = !kb_testing;
	            }
	     } else if (noisy_but == 1 && but == 0) {
	            noisy_but = 0;
	     } else if (but == 0 && noisy_but == 0) {
	            is_pressed_but = 0;
	          }

	        }
    if (output > 0) {

      if (tick >=
          (MAX_DURATION * scaler_speed)) {
        tick = 0;
        current_write_ptr++;
        if (current_write_ptr == writing_ptr) {
          current_write_ptr = 0;
        }
      }
      htim4.Instance->CCR2 = 100 * green[current_write_ptr].duration;
      htim4.Instance->CCR3 = 0;
      htim4.Instance->CCR4 = 0;
      if (red_yellow[current_write_ptr].color == RED_COLOR) {
        htim4.Instance->CCR4 = 100 * red_yellow[current_write_ptr].duration;
      } else {
        htim4.Instance->CCR3 = 100 * red_yellow[current_write_ptr].duration;
      }
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
