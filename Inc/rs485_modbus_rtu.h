#ifndef INC_RS485_MODBUS_RTU_H_
#define INC_RS485_MODBUS_RTU_H_

#include "main.h"
#include <string.h>

/*
 * Modbus command structure definition and buffer
 * */
typedef struct ModbusCommand {										// The structure of the 8-byte modbus command accepted by the module
	uint8_t		address;
	uint8_t		function_code;
	uint8_t		data[4];
	uint8_t		crc[2];
}ModbusCommand;

void MX_USART1_UART_Init(void);
void USART1_IRQHandler(void);					// USART1 global interrupt hanlder
void USART1_putchar(uint8_t ch);
void USART1_putstring(uint8_t *s);
uint8_t modbus_command_available(void);			// Check if there's a command in the buffer
ModbusCommand get_modbus_command(void);			// Fetch a modbus command from the buffer

#endif /* INC_RS485_MODBUS_RTU_H_ */
