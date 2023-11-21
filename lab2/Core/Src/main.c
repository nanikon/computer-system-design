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
#include "leds.h"
#include "morze.h"
#include "uart_irq.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LONG_PERIOD_CT 125
#define BETWEEN_LETTERS_PERIOD_CT 500
#define BUFFER_SIZE 2048
#define DEFAULT_DELAY 200
#define ERROR_DELAY 600
#define ERROR_COUNT 2
#define COMMA_DELAY DEFAULT_DELAY
#define DASH_DELAY 1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
int is_pressed = 0; // нажата ли кнопка, устанавливается в таймере
int is_wait_unpressed = 0; // ожидаем ли отпускание кнопки
int is_reading = 1; // есть ли что в памяти чтобы читать (можно pointer сделать глобальным и проверять на ноль
int writing_ptr = 0; //указатель на текущий конец сообщения
int count_tick = 0; // кол-во тиков таймера в текущем состоянии is_pressed, устанавливается в таймере
int noisy = 0; // доп проверка для дребезга
int has_irq = 0; //есть ли прерывания сейчас
int current_write_ptr = 0; // в каком месте буфера мы сейчас печатаем
int dash_delay = 0; // пауза тире
int comma_delay = 0; // пауза точки
int is_green = 0; // горит ли зелёный светодиод
int default_delay = 0; //пауза между символами


int buffer[BUFFER_SIZE] = {0}; //нули и единицы с кнопки
int buf_ptr = 0;

int int_w_buf[BUFFER_SIZE] = {0}; //конвертированные данные с uart
int int_w_ptr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t str = '2';
  has_irq = 0;

  while (1)
  {
	  int ret = receive_uart(&huart6, &str, has_irq);
	  if (ret > 0){ //есть что сконвертировать в 0 и 1
			char* res = char_to_morze(str);
			if (res[0] == '+') {
				has_irq = (has_irq + 1) % 2;
				if (has_irq == 1) {
					send_buffer_if_not_empty_IT(&huart6);
					NVIC_EnableIRQ(USART6_IRQn); // enable interrupts
				} else NVIC_DisableIRQ(USART6_IRQn); // disable interrupts
			}else if(res[0] == '-') {
	//			char i = '!';
	//			send_uart(&huart6, &i , 1, has_irq);
			}


		}

	  HAL_Delay(100);
	  send_buffer_if_not_empty(&huart6);
  }

  /*char char_w_buf[BUFFER_SIZE] = {0};
  int char_w_prt = 0;

  char char_r_buf[BUFFER_SIZE] = {0};
  int char_r_ptr = 0;
  
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *)&htim1);
  while (1){
	  recive_uart(&huart6, (char*)(char_r_buf + char_r_ptr), BUFFER_SIZE - char_r_ptr, has_irq); //читаем не пришло ли сообщение

	  if (char_r_ptr > 0){ //есть что сконвертировать в 0 и 1
      //TODO: disable irq
      int res = str_to_morze(char_r_buf, char_r_ptr, int_w_buf, int_w_ptr);
      char_r_ptr = 0; // прочитали весь буффер
      
      if (res == 0){ 
        has_irq = (has_irq + 1) % 2;
        if (has_irq == 1){
          send_buffer_if_not_empty_IT(&huart6);
          NVIC_EnableIRQ(USART6_IRQn); // enable interrupts
        }else NVIC_DisableIRQ(USART6_IRQn); // disable interrupts
      }
      writing_ptr = int_w_ptr;
      //TODO: enable irq
    }else if (0 == is_pressed && 0 == is_wait_unpressed) { //если кнопка не нажата
		  if (count_tick > LONG_PERIOD_CT && 0 != buf_ptr){ //после длинной паузы печатаем сообщение
			  send_uart(&huart6, char_w_buf, char_w_prt, has_irq);
        char_w_prt = 0;
		  }else if (count_tick > BETWEEN_LETTERS_PERIOD_CT){ // пришла длинная пауза после буквы
        //конвертируем очередную букву и кладём в буфер на отправку
        char_w_buf[char_w_prt] = morze_to_str(buffer, buf_ptr);
        char_w_prt++;
        buf_ptr = 0;
        if (char_w_prt == BUFFER_SIZE){
          send_uart(&huart6, char_w_buf, char_w_prt, has_irq);
          char_w_prt = 0;
        }
      }
	  } else if (0 == is_pressed && 1 == is_wait_unpressed) { //кнопку отпустили
		  is_wait_unpressed = 0;
       if (count_tick > LONG_PERIOD_CT) { // вносим 1 или 0 в буфер на отправку
			  turn_on_red_led();
			  HAL_Delay(DEFAULT_DELAY);
			  turn_off_red_led();
			  buffer[buf_ptr] = 1;
		  } else {
			  turn_on_yellow_led();
			  HAL_Delay(DEFAULT_DELAY);
			  turn_off_yellow_led();
			  buffer[buf_ptr] = 0;
		  }
		  count_tick = 0;
		  buf_ptr++;
		  if (BUFFER_SIZE == buf_ptr) {
			  for (int i = 0; i < ERROR_COUNT; i++) { //если буфер переполнится
				  turn_on_red_led();
				  turn_on_green_led();
				  HAL_Delay(ERROR_DELAY);
				  turn_off_red_led();
				  turn_off_green_led();
				  HAL_Delay(ERROR_DELAY);
			  }
        buf_ptr = 0;
		  }
	  } else if (1 == is_pressed && 0 == is_wait_unpressed) { //кнопку нажали -- ждём когда отпустят
		  is_wait_unpressed = 1;
		  count_tick = 0;
	  }

	  send_buffer_if_not_empty(&huart6);
  }*/
  
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
  htim1.Init.Prescaler = 16000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 15-1;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LEDG_Pin|LEDY_Pin|LEDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUT_Pin */
  GPIO_InitStruct.Pin = BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDG_Pin LEDY_Pin LEDR_Pin */
  GPIO_InitStruct.Pin = LEDG_Pin|LEDY_Pin|LEDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1){
		int but = HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin);
		but = !but;

		if(is_reading == 1){
			if(but == 1 && noisy == 0){
				noisy = 1;
			} else if(but == 1 && noisy == 1) {
				is_pressed = 1;
				count_tick++;
			} else if(noisy == 1 && but == 0){
				noisy = 0;
			} else if(but == 0 && noisy == 0){
				is_pressed = 0;
				count_tick++;
			}
		}

    if(writing_ptr > 0){ //выводим
			if(is_green){ //надо выключить
				if (dash_delay > 0){
					if (dash_delay < DASH_DELAY){ // ждём пока пауза не пройдёт
						dash_delay++;
					}else{ // пауза прошла -- меняем сигнал
						dash_delay = 0;
						turn_off_green_led();
						is_green = 0;
					}
				}else if(comma_delay > 0){
					if (comma_delay < COMMA_DELAY){ // ждём пока пауза не пройдёт
						comma_delay++;
					}else{ // пауза прошла -- меняем сигнал
						comma_delay = 0;
						turn_off_green_led();
						is_green = 0;
					}
				}
			}else{ //надо включить
				if (default_delay > DEFAULT_DELAY){ // ждём следующего момента выводить?
					// зажигаем зелёный цвет и включаем флаги ожидания
					default_delay = 0;
					turn_on_green_led();
					is_green = 1;
					if(int_w_buf[current_write_ptr] == 1){
						dash_delay = 1;
					}else{
						comma_delay = 1;
					}
				}else{ // пока ещё ждём
					default_delay++;
				}
				
			}

			if(current_write_ptr == writing_ptr){ // TODO: проверить, что прерывания не ломают writing_ptr
        current_write_ptr = 0;
        writing_ptr = 0;
      }
			current_write_ptr++;
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
