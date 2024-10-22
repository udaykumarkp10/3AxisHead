/*
 * br24l64f.c
 *
 *  Created on: Aug 21, 2024
 *      Author: Rajdeep
 */
#include "br24l64f.h"

// This function helps to write into EEPROM
void EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size) {
    HAL_I2C_Mem_Write(&EEPROM_PORT, EEPROM_ADDRESS_WRITE, addr, I2C_MEMADD_SIZE_16BIT, data, size, HAL_MAX_DELAY);
    HAL_Delay(5);
}

// This function helps to read from EEPROM
void EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size) {
    HAL_I2C_Mem_Read(&EEPROM_PORT, EEPROM_ADDRESS_READ, addr, I2C_MEMADD_SIZE_16BIT, data, size, HAL_MAX_DELAY);
    HAL_Delay(5);
}

void float2Bytes(uint8_t *bytes_data, float variable){
    union {
    	float f_data;
    	uint8_t b_data[4];
    }DataMem;

    DataMem.f_data = variable;

    for (uint8_t i = 0; i < 4; i++) {
      bytes_data[i] = DataMem.b_data[i];
    }
}

float bytes2Float(uint8_t *bytes_data){
    union {
    	float f_data;
    	uint8_t b_data[4];
    }DataMem;

    for (uint8_t i = 0; i < 4; i++) {
    	DataMem.b_data[i] = bytes_data[i];
    }

   float variable =  DataMem.f_data;
   return variable;
}

void uInt2Bytes(uint8_t *bytes_data, uint32_t variable){
    union {
    	uint32_t f_data;
    	uint8_t b_data[4];
    }DataMem;

    DataMem.f_data = variable;

    for (uint8_t i = 0; i < 4; i++) {
      bytes_data[i] = DataMem.b_data[i];
    }
}
uint32_t bytes2uInt(uint8_t *bytes_data){
    union {
      uint32_t f_data;
      uint8_t b_data[4];
    }DataMem;

    for (uint8_t i = 0; i < 4; i++) {
    	DataMem.b_data[i] = bytes_data[i];
    }

   uint32_t variable =  DataMem.f_data;
   return variable;
}

void sInt2Bytes(uint8_t *bytes_data, int32_t variable){
    union {
      int32_t f_data;
      uint8_t b_data[4];
    }DataMem;

    DataMem.f_data = variable;

    for (uint8_t i = 0; i < 4; i++) {
      bytes_data[i] = DataMem.b_data[i];
    }
}
int32_t bytes2sInt(uint8_t *bytes_data){
    union {
      int32_t f_data;
      uint8_t b_data[4];
    }DataMem;

    for (uint8_t i = 0; i < 4; i++) {
    	DataMem.b_data[i] = bytes_data[i];
    }

   int32_t variable =  DataMem.f_data;
   return variable;
}
