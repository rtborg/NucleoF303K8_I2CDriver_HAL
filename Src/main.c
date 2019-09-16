#include "main.h"
#include <string.h>
#include "sfm4100_driver.h"

// Peripheral handles as generated by Cube
I2C_HandleTypeDef hi2c1;// Note: sfm4100_driver.c uses this variable as extern in order to use the I2C
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// Functions generated by Cube
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);

// Various user functions
void USART1_IRQHandler(void);					// USART1 global interrupt hanlder
void USART1_putchar(uint8_t ch);
void USART1_putstring(uint8_t *s);

// SFM4100 variables
uint8_t sfm4100_error = 0;
uint16_t sfm4100_register_value = 0;
uint32_t sfm4100_serial_number = 0;

// Modbus data buffer. When receiving, the UART1 IRQ is filling in the buffer
volatile uint8_t command_flag = 0;				// Flag raised if a 6-byte command is received by the USART


uint8_t uart_buffer[64] = {0};					// UART1 transmit buffer

/*
 * UART1 Inettupt based-transmit buffer
 * */
#define UART1_TX_BUFFER_SIZE 64
volatile uint8_t uart1TxHead = 0;
volatile uint8_t uart1TxTail = 0;
volatile uint8_t uart1TxBuffer[UART1_TX_BUFFER_SIZE];
volatile uint8_t uart1TxBufferRemaining;

/*
 * Modbus buffer - 8 bytes, populated by USART1 IRQ
 * */
#define MODBUS_COMMAND_LENGTH	8
volatile uint8_t modbus_buffer_head = 0;
volatile uint8_t modbus_buffer_tail = 0;
volatile uint8_t modbus_rx_buffer[MODBUS_COMMAND_LENGTH];
volatile uint modbus_buffer_count = 0;



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
	USART1_putstring(uart_buffer);														// Print serial no.

	while (1) {

		HAL_Delay(250);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		/*
		 * Check if a command has been received
		 * */
		if (command_flag) {
			if (modbus_rx_buffer[0] == 0x01 && modbus_rx_buffer[1] == 0x04) {			// Check if address and function code are correct
				sfm4100_error = 0;														// Clear error
				sfm4100_register_value = 0;												// Clear temporary register value
				sfm4100_error = sfm4100_measure(FLOW, &sfm4100_register_value);			// Trigger flow measurement
				sprintf(uart_buffer, "GAS FLOW: %d sccm\n\r", sfm4100_register_value);	// Copy returned flow measurement into buffer
				USART1_putstring(uart_buffer);											// Print measurement
				command_flag = 0;														// Set command flag to zero
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

	uart1TxHead = 0;												// Initialize UART buffer variables
	uart1TxTail = 0;
	uart1TxBufferRemaining = sizeof(uart1TxBuffer);

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

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {											// Handle RX interrupt
		huart1.Instance->RTOR |= 0x50;															// Test only: set RTOR again after each reception

		modbus_rx_buffer[modbus_buffer_head++] = USART1->RDR;									// Place char in modbus command buffer (8 bytes only)
		if (modbus_buffer_head == MODBUS_COMMAND_LENGTH) modbus_buffer_head = 0;				// Wrap-around buffer head
		modbus_buffer_count++;																	// Increase modbus buffer count
	}

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {					// Clear overrun flag
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
	}

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {					// Handle transmit interrupt
		if(sizeof(uart1TxBuffer) > uart1TxBufferRemaining) {			// If the number of free spaces in the buffer is less than the size of the buffer that means there's still characters to be sent
			USART1->TDR = uart1TxBuffer[uart1TxTail++];					// Place char in the TX buffer. This also clears the interrupt flag
			if(sizeof(uart1TxBuffer) <= uart1TxTail)					// Wrap around tail if needed
			{
				uart1TxTail = 0;
			}
			uart1TxBufferRemaining++;			 						// Increase number of remaining characters
		} else {														// If remaining chars == buffer size, there's nothing to transmit
			USART1->CR1 &= ~USART_CR1_TXEIE;							// Disable TXE interrupt
			__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TXE);				// Clear flag
		}
	}


	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RTOF)) {					// The Receive Timeout interrupt happens when an idle tie of more than 40 bits (3.5 modbus 11 bit chars)
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RTOF);					// Clear receive timeout interrupt flag

		if (modbus_buffer_head == 0 && modbus_buffer_count == 8) {		// Check modbus command buffer. If head == 0 and count == 8, an 8-byte command has been received and head has wrapped around
			command_flag = 1;											// Set command flag
			modbus_buffer_count = 0;									// Zero modbus command buffer count
		} else {														// If counter is not 8 and head is not wrapped around, clear the buffer
			modbus_buffer_head = 0;										// and wait for another modbus message
			modbus_buffer_count = 0;
		}
	}
}

/**
 * USART1 interrupt-driven putchar function
 * @param ch
 */
void USART1_putchar(uint8_t ch) {
	while (0 == uart1TxBufferRemaining) continue;						// Wait until there's a free space in the transmit buffer

	if (0 == (USART1->CR1 & USART_CR1_TXEIE)) {							// If TXE interrupt is disabled, directly put char in USART TDR
		USART1->TDR = ch;
	} else {															// If TXE interrupt is enabled, there's transmission going on
		USART1->CR1 &= ~USART_CR1_TXEIE;								// Disable TXE interrupt temporarily
		uart1TxBuffer[uart1TxHead++] = ch;								// Place data in buffer
		if(sizeof(uart1TxBuffer) <= uart1TxHead)						// Wrap around buffer head
		{
			uart1TxHead = 0;
		}
		uart1TxBufferRemaining--;										// Decrease number of free spaces in buffer
	}

	USART1->CR1 |= USART_CR1_TXEIE;										// Enable TXE interrupt
}

/**
 * USART1 interrupt-driven putstring function
 * @param s
 */
void USART1_putstring(uint8_t *s) {
	uint ar_size = strlen(s);

	for (int i = 0; i < ar_size; i++) {
		USART1_putchar(s[i]);
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
