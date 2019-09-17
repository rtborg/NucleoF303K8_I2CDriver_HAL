/*
 * sfm4100_driver.h
 *
 *  Created on: 10 Sep 2019
 *      Author: 721850
 */

#ifndef INC_SFM4100_DRIVER_H_
#define INC_SFM4100_DRIVER_H_

#include "main.h"

// Sensiron SFM4100 address is actually 0x01 shifted to the left by 1 bit
// The datasheet wrongly shows bit 7 of the address field set
#define SENSOR_ADDR		(uint16_t)(0x01 << 1)
#define POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001

// Measurement types
typedef enum {
	FLOW					= 0xf1,
	TEMP 					= 0xf3,
	VDD						= 0xf5,
}SFM4100_Measurement_Type;

// SF04 eeprom map
#define EE_ADR_PART_NAME 0x02E8		// 10-byte array
#define EE_ADR_SN_PRODUCT 0x02F8	// 32-bit uint
#define EE_ADR_SCALE_FACTOR 0x02B6
#define EE_ADR_FLOW_UNIT 0x02B7

// SFM4100 API
void sfm4100_init(void);
uint8_t sfm4100_get_eeprom_base_address(uint16_t *p_address);
uint8_t sfm4100_read_register(uint8_t reg, uint16_t *p_register_value);
uint8_t sfm4100_write_register(uint8_t reg, uint16_t *p_register_value);
uint8_t sfm4100_read_serial_number(uint32_t *p_serial_number);
uint8_t sfm4100_soft_reset();
uint8_t sfm4100_read_eeprom(uint16_t eeprom_start_address, uint8_t size, uint8_t eeprom_data[]);
uint8_t sfm4100_check_crc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
uint8_t sfm4100_measure(SFM4100_Measurement_Type measurement_type, uint16_t *p_address);


#endif /* INC_SFM4100_DRIVER_H_ */
