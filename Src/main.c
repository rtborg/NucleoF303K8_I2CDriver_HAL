#include "main.h"
#include <string.h>
#include "sfm4100_driver.h"
#include "FreeRTOS.h"
#include "task.h"


/*
 * Autogenerated peropheral inits
 * */
I2C_HandleTypeDef hi2c1;						// Note: sfm4100_driver.c uses this variable as extern in order to use the I2C
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);

/*
 * User generated functions
 * */
void USART1_IRQHandler(void);								// USART1 global interrupt hanlder
void usart1_print_msg(UART_HandleTypeDef *huart, char *msg);							// Print a message via uart 1 using polling
void vKeepaliveLEDHandler(void *params); 					// Task to blink the on-board LED
void vCommandProcessingHandler(void *params); 				// Task to process Modbus commands

/* FreeRTOS variables */
TaskHandle_t xTaskHandleKeepaliveLED = NULL;
TaskHandle_t xTaskHandleCommandProcessing = NULL;

// SFM4100 variables
uint8_t sfm4100_error = 0;
uint16_t sfm4100_register_value = 0;
uint32_t sfm4100_serial_number = 0;

// Modbus data buffer. When receiving, the UART1 IRQ is filling in the buffer
volatile char modbus_buffer[256] = { 0 };		// Modbus data buffer. Commands will be no more than 8 bytes long
volatile uint8_t m_buffer_index = 0;			// Index of modbus buffer
volatile uint8_t command_flag = 0;				// Flag raised if a 6-byte command is received by the USART

// Message
char *msg = "Hello World!\n\r";
char uart_buffer[64] = {0};

int main(void) {

	// Initialize peripherals
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();

	// @TODO Use a command queue
	// @TODO CRC check

	sfm4100_soft_reset();																// Issue soft reset
	uint8_t err = sfm4100_read_serial_number(&sfm4100_serial_number);					// Get device serial number
	sprintf(uart_buffer, "Sensor serial no: %d\n\r", sfm4100_serial_number);			// Print serial number to the console
	HAL_UART_Transmit(&huart1, uart_buffer, strlen(uart_buffer), 1000);

	// Create tasks
	xTaskCreate(vKeepaliveLEDHandler, "KeepaliveLED", configMINIMAL_STACK_SIZE, NULL, 2,
			&xTaskHandleKeepaliveLED);
	xTaskCreate(vCommandProcessingHandler, "CommProcessing", configMINIMAL_STACK_SIZE, NULL, 2,
			&xTaskHandleCommandProcessing);
	// Start the scheduler
	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler */

	while (1) {

		/*
		 * Check if a command has been received
		 * */
		if (command_flag) {
			if (modbus_buffer[0] == 0x01 && modbus_buffer[1] == 0x04) {					// Check if address and function code are correct
				sfm4100_error = 0;														// Clear error
				sfm4100_register_value = 0;												// Clear temporary register value
				sfm4100_error = sfm4100_measure(FLOW, &sfm4100_register_value);			// Trigger flow measurement
				sprintf(uart_buffer, "GAS FLOW: %d sccm\n\r", sfm4100_register_value);	// Copy returned flow measurement into buffer
				HAL_UART_Transmit(&huart1, uart_buffer, strlen(uart_buffer), 1000);		// Send buffer via uart1 (modbus)
				command_flag = 0;
			}


		}

	}

}
/****************************************************************************************************************/
/**
 * @brief System Clock Configuration
 * @retval None
 */
/****************************************************************************************************************/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/****************************************************************************************************************/
/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
static void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00201D2B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
}

/****************************************************************************************************************/
/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/****************************************************************************************************************/
/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_2;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 16, 16) != HAL_OK) {
		Error_Handler();
	}

	huart1.Instance->CR2 |= USART_CR2_RTOEN;						// Enable receiver timeout for Modbus
	huart1.Instance->RTOR |= 0x50;									// Timeout: 40 bits. 1 bit @ 9600bps = 1/9600 = 104.17us. 40 bits = 4.17ms, approx. 3.5 chars * 11 bits each

	// Set interrupt priority & enable interrupts
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 5);						// Set interrupt priority
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RTO);						// Enable Receive Timeout interrupt
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);					// Enable Receive interrupt
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/****************************************************************************************************************/
/**
 * @brief GPIO Initialization Function. LD3 is on-board LED; Debug_Pin is PB4 and is used for various debug
 * purposes
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Debug_Pin_Port, Debug_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LD3_Pin and Debug_Pin*/
	GPIO_InitStruct.Pin = LD3_Pin | Debug_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/****************************************************************************************************************/
/**
 * USART1 interrupt service routine
 */
/****************************************************************************************************************/
void USART1_IRQHandler(void) {

	/*
	 * Handle receive interrupt. Buffer size is 256 bytes and indexer is an unsigned 8-bit variable
	 * which will overflow if message received is larger than 256 bytes
	 * */
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
		modbus_buffer[m_buffer_index++] = USART1->RDR;
		// Test only: set RTOR again after each reception
		huart1.Instance->RTOR |= 0x50;
	}

	/*
	 * Clear overrun flag
	 * */
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
	}

	/* The Receive Timeout interrupt happens when an idle tie of more than 40 bits (3.5 modbus 11 bit chars)
	 * is detected. */
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RTOF)) {
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RTOF);					// Clear receive timeout interrupt flag
		if (m_buffer_index == 6) {										// If 6 bytes are received, treat message as potential command
			command_flag = 1;											// Raise flag for command received
			m_buffer_index = 0;
		} else {														// If the message is not 6 bytes, discard it
			m_buffer_index = 0;
		}

	}
}

/**
 * Send data to UART using polling
 * @note The function waits until UART_ISR_TC is set and can wait indefinitely!
 * @param huart
 * @param msg
 */
void usart1_print_msg(UART_HandleTypeDef *huart, char *msg) {
	int message_length = strlen(msg);
	for (int i = 0; i < message_length; i++) {
		huart->Instance->TDR = (msg[i] & (uint8_t)0xFFU);
		while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == 0) continue;
	}
}

/**
 * Blink the on-board LED
 */
void vKeepaliveLEDHandler(void *params) {
	while (1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		vTaskDelay(250);
	}
}

/**
 * Process Modbus commands
 */
void vCommandProcessingHandler(void *params) {
	while (1) {
		usart1_print_msg(&huart1, "Hello from FreeRTOS!\n\r");
		vTaskDelay(1000);
	}
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM2) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
