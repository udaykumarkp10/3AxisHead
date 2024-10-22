/*
 * prj_settings.h
 *
 *  Created on: Aug 14, 2024
 *      Author: Rajdeep
 */

#ifndef INC_PROJECT_H_
#define INC_PROJECT_H_

#include "main.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "velocity_profile.h"
#include "stdint.h"

#define TMC_PORT 		hspi1	//update this based on hardware
#define EEPROM_PORT		hi2c2	//update this based on hardware
#define PCAP_PORT		hi2c1
#define ADXL345_PORT	hspi2
#define LAN9252_PORT	hspi5

#define PCAP_TIMER		htim4

extern SPI_HandleTypeDef 	TMC_PORT;
extern I2C_HandleTypeDef 	EEPROM_PORT;
extern TIM_HandleTypeDef	PCAP_TIMER;
extern I2C_HandleTypeDef 	PCAP_PORT;
extern SPI_HandleTypeDef 	ADXL345_PORT;
extern SPI_HandleTypeDef 	LAN9252_PORT;
extern TIM_HandleTypeDef	PCAP_TIMER;

extern volatile bool index_found;

#define TMC4671_CS_SET() 	HAL_GPIO_WritePin(CS_TMC4671_GPIO_Port, CS_TMC4671_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(CS_TMC6100_GPIO_Port, CS_TMC6100_Pin, GPIO_PIN_SET)
#define TMC4671_CS_RESET() 	HAL_GPIO_WritePin(CS_TMC4671_GPIO_Port, CS_TMC4671_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(CS_TMC6100_GPIO_Port, CS_TMC6100_Pin, GPIO_PIN_SET)

#define TMC6100_CS_SET() 	HAL_GPIO_WritePin(CS_TMC4671_GPIO_Port, CS_TMC4671_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(CS_TMC6100_GPIO_Port, CS_TMC6100_Pin, GPIO_PIN_RESET)
#define TMC6100_CS_RESET() 	HAL_GPIO_WritePin(CS_TMC4671_GPIO_Port, CS_TMC4671_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(CS_TMC6100_GPIO_Port, CS_TMC6100_Pin, GPIO_PIN_SET)

#define LAN9252_CS_SET() 	HAL_GPIO_WritePin(LAN9252_CS_GPIO_Port, LAN9252_CS_Pin, GPIO_PIN_RESET)
#define LAN9252_CS_RESET() 	HAL_GPIO_WritePin(LAN9252_CS_GPIO_Port, LAN9252_CS_Pin, GPIO_PIN_SET)

#define ADXL345_CS_SET() 	HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_SET)
#define ADXL345_CS_RESET() 	HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_RESET)

#endif /* INC_PROJECT_H_ */
