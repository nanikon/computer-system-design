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
#define WRITE_BUFFER_SIZE 512
#define MIDDLE_BUFFER_SIZE 100
#define TICK_BUFF_SIZE 20
#define MAX_DURATION 10
#define DEFAULT_LIGHT_DURATION 10
#define MODES_COUNT 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t read_buffer = 0;
uint8_t output = 1; // сейчас читаем или выводим
Tick green[TICK_BUFF_SIZE] = {};
Tick red_yellow[TICK_BUFF_SIZE] = {};
uint8_t tick = 0;
uint32_t writing_ptr;
uint32_t current_write_ptr;
Mode modes[MODES_COUNT] = {0};

uint8_t buffer_to_write[WRITE_BUFFER_SIZE] = {}; // буфер на передачу
uint8_t middle_buffer[MIDDLE_BUFFER_SIZE] = {};
size_t start_write = 0; // номер символа с которого начинать передачу
size_t end_write = 0;

uint8_t mode = 1;
uint32_t min_scaler = 1;
uint32_t max_scaler = 100;
uint32_t scaler_speed = 10;
uint32_t new_scaler_speed;

Input_tick tick_buffer[TICK_BUFF_SIZE] = {};
Input_tick cur_read_tick = {0};
uint8_t tick_ptr = 0;
uint8_t tick_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
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
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)&htim1);
  HAL_UART_Receive_IT(&huart6, (uint8_t*) &read_buffer, 1);
  init_modes(modes);
  play_new_mode(&modes[0], green, red_yellow, &writing_ptr, &current_write_ptr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  htim1.Init.Period = 100-1;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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

void send_uart(UART_HandleTypeDef *huart, uint8_t* buffer, size_t buf_size) {
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
		send_buffer_if_not_empty_IT(huart);
	}
}

void send_user_mode_param();

void fill_softnes_and_save_input_tick(uint8_t softness) {
	tick_ptr = 0;
	cur_read_tick.softness = softness;
	memcpy(tick_buffer + tick_len, &cur_read_tick, sizeof(Input_tick));
	tick_len++;
}

void handler_input() {
	if (output == 1) {
		switch (read_buffer) {
		case '1':
			// to next mode
			if (mode == 5) {
				mode = 1;
				play_new_mode(&modes[mode - 1], green, red_yellow, &writing_ptr, &current_write_ptr);
			} else {
				mode++;
				play_new_mode(&modes[mode - 1], green, red_yellow, &writing_ptr, &current_write_ptr);
			}
			break;
		case '2':
			// to previos mode
			if (mode == 1) {
				mode = 5;
				play_new_mode(&modes[mode - 1], green, red_yellow, &writing_ptr, &current_write_ptr);
			} else {
				mode--;
				play_new_mode(&modes[mode - 1], green, red_yellow, &writing_ptr, &current_write_ptr);
			}
			break;
		case '3':
			// faster
			new_scaler_speed = scaler_speed - round((double) scaler_speed / 10);
			if (new_scaler_speed == scaler_speed) {
				new_scaler_speed++;
			}
			if (new_scaler_speed > min_scaler) {
				scaler_speed = new_scaler_speed;
			}
			break;
		case '4':
			// slower
			new_scaler_speed = scaler_speed + round((double) scaler_speed / 10);
			if (new_scaler_speed < max_scaler) {
				scaler_speed = new_scaler_speed;
			}
			break;
		case '5':
			// send current user mode
			//send_user_mode_param();
			break;
		case '\n':
			// ввод в меню настройки
			output = 0;
			break;
		default:
			break;
		}
	} else {
		if (read_buffer == '\n') {
			if (tick_len != 0) {
				fill_mode_array(tick_buffer, tick_len, green, red_yellow, &writing_ptr);
				tick_len = 0;
			}
			output = 1;
		} else {
			switch (tick_ptr) {
				case 0:
					if (tick_len == TICK_BUFF_SIZE) {
						bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
						sprintf((char*) middle_buffer, "user mode buffer is full!");
						send_uart(&huart6, middle_buffer, strlen((char*)middle_buffer));
					} else {
						if (read_buffer == 'r' || read_buffer == 'g') {
							tick_ptr++;
							cur_read_tick.color = RED_COLOR;
						} else if (read_buffer == 'y') {
							tick_ptr++;
							cur_read_tick.color = 0;
						}
					}
					break;
				case 1:
					if (read_buffer >= '1' && read_buffer <= '9'){
						tick_ptr++;
						cur_read_tick.brightness = read_buffer - '0';
					}
					break;
				case 2:
					if (read_buffer >= '1' && read_buffer <= '9'){
						tick_ptr++;
						cur_read_tick.duration = read_buffer - '0';
					}
					break;
				case 3:
					if (read_buffer == '+') {
						fill_softnes_and_save_input_tick(1);
					} else if (read_buffer == '-') {
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    // USART6 завершил прием данных
	  send_uart(huart, (uint8_t*) &read_buffer, sizeof(uint8_t));
	  handler_input();
	  if (output == 1) {
		  // режим проигрывания - выводим номер мелодии и скорость
		  bzero(middle_buffer, MIDDLE_BUFFER_SIZE);
		  sprintf((char*) middle_buffer, "mode: %d, speed: %ld", mode, scaler_speed);
		  send_uart(huart, middle_buffer, strlen((char*)middle_buffer));
	  } else {
		  // режим пользовательской конфигурации - выводим ту которая есть сейчас
		  // send current user configuration
		  //send_user_mode_param();
	  }
	  HAL_UART_Receive_IT(huart, (uint8_t*) &read_buffer, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART6)
  {
      // USART6 завершил отправку данных
	  send_buffer_if_not_empty_IT(huart);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM1){
	  if (output > 0) {
		  tick++;
		  if (tick >= (MAX_DURATION * scaler_speed)) { // scaler как-то сюда добавлять
		     tick = 0;
		     current_write_ptr++;
		     if (current_write_ptr == writing_ptr) {
		       current_write_ptr = 0;
		     }
		     /*middle_buffer[0] = red_yellow[current_write_ptr].duration + '0';
		     middle_buffer[1] = ' ';
		     middle_buffer[2] = red_yellow[current_write_ptr].color + '0';
		     middle_buffer[3] = ' ';
		     middle_buffer[4] = current_write_ptr + '0';
		     middle_buffer[5] = ';';
		     send_uart(&huart6, middle_buffer, 6 * sizeof(uint8_t));*/
		  }
		  htim4.Instance->CCR2 = 100 * green[current_write_ptr].duration;
		  htim4.Instance->CCR3 = 0;
		  htim4.Instance->CCR4 = 0;
		  if (red_yellow[current_write_ptr].color == RED_COLOR) {
			  htim4.Instance->CCR4 = 100 * red_yellow[current_write_ptr].duration;
			  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			  //middle_buffer[0] = 'r';
			  //send_uart(&huart6, middle_buffer, sizeof(uint8_t));
		  } else {
			  htim4.Instance->CCR3 = 100 * red_yellow[current_write_ptr].duration;
			  //middle_buffer[0] = 'y';
			  //send_uart(&huart6, middle_buffer, sizeof(uint8_t));
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
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
