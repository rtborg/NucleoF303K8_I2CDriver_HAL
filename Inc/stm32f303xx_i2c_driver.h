#include "stm32f303x8.h"

#ifndef INC_STM32F303XX_I2C_DRIVER_H_
#define INC_STM32F303XX_I2C_DRIVER_H_

/**
 * Configuration structure for the I2C peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;
}I2C_Config_t;


typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/**
 * @I2C_SCLSpeed
 */
#define I2C_SCK_SPEED_SM	1000000
#define I2C_SCL_SPEED_FM	4000000

/**
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/**
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16by9	1


#endif /* INC_STM32F303XX_I2C_DRIVER_H_ */
