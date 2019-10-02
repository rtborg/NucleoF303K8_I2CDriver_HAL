/*
 * sfm4100_driver.c
 *
 *  Created on: 10 Sep 2019
 *      Author: 721850
 */

#include "sfm4100_driver.h"

static I2C_HandleTypeDef hi2c1;

/**
 * SFM4100 user commands list
 * Each SFM4100 command returns or expects 2 bytes with the exception of the last 3
 */
uint8_t user_reg_w = 0xe2;	// 16 bits
uint8_t user_reg_r = 0xe3; // 16 bits
uint8_t adv_user_reg_w = 0xe4; // 16 bits
uint8_t adv_user_reg_r = 0xe5; // 16 bits
uint8_t read_only_reg1_r = 0xe7;
uint8_t read_only_reg2_r = 0xe9;
uint8_t trigger_flow_measurement = 0xf1;
uint8_t trigger_temp_measurement = 0xf3;
uint8_t trigger_vdd_measurement = 0xf5; // Not implemented on SFM4100
uint8_t eeprom_w = 0xfa;
uint8_t eeprom_r = 0xfa;
uint8_t soft_reset = 0xfe;

// SFM4100 variables
uint8_t sfm4100_data_buffer[32] = { 0 };		// Receive buffer for SFM4100 functions

/****************************************************************************************************************/
/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
/****************************************************************************************************************/
void sfm4100_init(void) {
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
 * SFM4100 function for calculating EEPROM base address
 * @param p_address A pointer which will be set to the EEPROM base address
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_get_eeprom_base_address(uint16_t *p_address) {
	uint8_t error = 0;
	error |= sfm4100_read_register(read_only_reg2_r, p_address);

	*p_address &= 0x07;
	*p_address *= 0x300;

	return error;
	/**
	 * The EEPROM address is composed of an EEPROM base address and an address offset.
	 * To determine the base EEPROM address for general sensor information,
	 * bit <2:0> of Read-Only Register 2 (called active configuration field at boot time)
	 * must be multiplied by h300.
	 * Then the Wordadr Offset of following table must be added to that address.
	 * See Application Note for I2C Flow and Differential Pressure Sensors
	 */
}

/****************************************************************************************************************/
/**
 * SFM4100 read register function
 * @param reg The address of the register as defined in this file
 * @param p_register_value The contents of the register will be copied in this pointer
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_read_register(uint8_t reg, uint16_t *p_register_value) {
	uint8_t error = 0;
	uint16_t reg_contents = 0;
	uint8_t temp_buffer[8] = {0};

	error |= HAL_I2C_Master_Transmit(&hi2c1, SENSOR_ADDR, &reg, 0x01, 1000);						// Issue write command
	error |= HAL_I2C_Master_Receive(&hi2c1, SENSOR_ADDR, temp_buffer, 0x03, 1000);					// Read contents of register
	error |= sfm4100_check_crc(temp_buffer, 2, temp_buffer[2]); 							// Check CRC

	reg_contents |= (uint16_t) (temp_buffer[0] << 8);
	reg_contents |= (uint16_t) (temp_buffer[1]);
	*p_register_value = reg_contents;

	return error;
}

/****************************************************************************************************************/
/**
 * SFM4100 write register function
 * @param reg The address of the register as defined in this file
 * @param p_register_value The value to be written in the register
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_write_register(uint8_t reg, uint16_t *p_register_value) {
	// A write to a register will be 2 bytes only. Need to concatenate address and reg value
	// Total transfer length: 3 bytes
	uint8_t error = 0;
	uint16_t register_value = *p_register_value;
	uint8_t data[3] = { 0 };
	data[0] = reg;
	data[1] = (uint8_t) (register_value >> 8);
	data[2] = (uint8_t) (register_value & 0xff);

	error |= HAL_I2C_Master_Transmit(&hi2c1, SENSOR_ADDR, data, 0x03, 1000);
	return error;
}

/****************************************************************************************************************/
/**
 * SFM4100 Read serial number of the device
 * @param p_serial_number The serial number will be written into this pointer
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_read_serial_number(uint32_t *p_serial_number) {
	uint8_t error = 0;
	uint16_t eeprom_base_address = 0;
	uint16_t eeprom_address = 0;
	uint32_t serial_number = 0;

	error = sfm4100_get_eeprom_base_address(&eeprom_base_address);					// Get eeprom base address
	eeprom_address = eeprom_base_address + EE_ADR_SN_PRODUCT;						// Get serial number field address in eeprom
	error |= sfm4100_read_eeprom(eeprom_address, 6, sfm4100_data_buffer);			// Read serial number: 2 bytes - CRC - 2 bytes - CRC, total of 6 bytes
	error |= sfm4100_check_crc(sfm4100_data_buffer, 2, sfm4100_data_buffer[2]);		// Check first CRC
	uint8_t *p = &sfm4100_data_buffer[3];											// The second 16-bit register starts at sfm4100_data_buffer[3]
	error |= sfm4100_check_crc(p, 2, p[2]);											// Check second CRC

	serial_number |= (sfm4100_data_buffer[0] << 24);								// Copy the 4 received bytes into the function argument
	serial_number |= (sfm4100_data_buffer[1] << 16);
	serial_number |= (sfm4100_data_buffer[3] << 8);
	serial_number |= (sfm4100_data_buffer[4] << 0);
	*p_serial_number = serial_number;

	return error;
}

/****************************************************************************************************************/
/**
 * SFM4100 Soft reset
 * The function configures the sensor for 16-bit flow measurement resolution
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_soft_reset() {
	uint8_t error = 0;
	uint16_t sfm4100_register_value = 0;
	uint16_t data = 0;

	error |= HAL_I2C_Master_Transmit(&hi2c1, SENSOR_ADDR, &soft_reset, 0x01, 1000);		// Issue soft reset
	HAL_Delay(32);																		// Wait - see LQ_AN_LiquidFlowSensors I2C implementation
	error |= sfm4100_read_register(trigger_flow_measurement, &data);					// Get a dummy read

	error |= sfm4100_read_register(adv_user_reg_r, &sfm4100_register_value); 			// Get Adv User Register
	sfm4100_register_value |= 0xe00;													// Change flow resolution to 16 bits
	error |= sfm4100_write_register(adv_user_reg_w, &sfm4100_register_value);			// Write new value to Adv User Register
	error |= sfm4100_read_register(adv_user_reg_r, &sfm4100_register_value); 			// Get Adv User Register to confirm resolution

	return error;
}

/****************************************************************************************************************/
/**
 * SFM4100 read EEPROM data
 * @param eeprom_start_address
 * @param size - the number of bytes to read
 * @param eeprom_data - pointer to an array of bytes into which the EEPROM data will be copied
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_read_eeprom(uint16_t eeprom_start_address, uint8_t size, uint8_t eeprom_data[]) {
	uint8_t error = 0;
	uint8_t eeprom_address[3] = {0};														// Create array from read eerpom command and the eeprom address
	eeprom_start_address = ((eeprom_start_address << 4) & 0xfff0);							// Left-shift the eeprom address by 4 bits
	eeprom_address[0] = eeprom_r;															// Command 0xFA
	eeprom_address[1] = (uint8_t) (eeprom_start_address >> 8) & 0xff;						// MSB of the eeprom address
	eeprom_address[2] = (uint8_t) (eeprom_start_address & 0xff);							// LSB of the eeprom address

	error |= HAL_I2C_Master_Transmit(&hi2c1, SENSOR_ADDR, eeprom_address, 0x03, 1000);		// Send command
	error |= HAL_I2C_Master_Receive(&hi2c1, SENSOR_ADDR, eeprom_data, size, 1000);			// Get eeprom data and copy it into the user-supplied array

	return error;
}

/****************************************************************************************************************/
/**
 * Calculate CRC of received data. Function adapred from Sensirion Mass Fwol Meters CRC Calculation application note
 * @param data - checksum is built based on this data
 * @param nbrOfBytes = checksum is built for n bytes of data
 * @param checksum - expected checksum
 * @return 0 on success
 */
/****************************************************************************************************************/
uint8_t sfm4100_check_crc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum) {
	uint8_t crc = 0;
	uint8_t byteCtr;
	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) {
		crc ^= (data[byteCtr]);
		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc != checksum) return 1;
	else return 0;
}

/****************************************************************************************************************/
/**
 * Measure flow, temperature or voltage (voltage measurement is not implemented on SFM4100)
 * @param measurement_type FLOW, TEMP or VDD
 * @param p_address Pointer to the 16-bit variable to which the data will be written
 * @return
 */
/****************************************************************************************************************/
uint16_t sfm4100_measure(SFM4100_Measurement_Type measurement_type,
		uint16_t *p_address) {
	uint8_t error = 0;
	uint16_t data = 0;

	switch (measurement_type) {
	case FLOW:
		error |= sfm4100_read_register(trigger_flow_measurement, &data);
		break;
	case TEMP:
		error |= sfm4100_read_register(trigger_temp_measurement, &data);
		break;
	case VDD:
		error |= sfm4100_read_register(trigger_temp_measurement, &data);
		break;
	default:
		break;
	}

	*p_address = data;
	return error;
}
