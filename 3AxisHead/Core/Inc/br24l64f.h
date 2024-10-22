/*
 * br24l64f.h
 *
 *  Created on: Aug 21, 2024
 *      Author: Rajdeep
 */

#ifndef INC_BR24L64F_H_
#define INC_BR24L64F_H_

#include <project.h>

#define EEPROM_ADDRESS_WRITE  0xA0    // Write Address
#define EEPROM_ADDRESS_READ   0xA1    //  Read Address

void EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size);
void EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size);

void uInt2Bytes(uint8_t *bytes_data, uint32_t variable);
uint32_t bytes2uInt(uint8_t *bytes_data);

void sInt2Bytes(uint8_t *bytes_data, int32_t variable);
int32_t bytes2sInt(uint8_t *bytes_data);

void float2Bytes(uint8_t *bytes_data, float float_variable);
float bytes2Float(uint8_t *bytes_data);

#endif /* INC_BR24L64F_H_ */
