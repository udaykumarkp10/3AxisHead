/*
 * tmc6100.h
 *
 *  Created on: May 13, 2024
 *      Author: Rajdeep
 */

#ifndef INC_TMC6100_H_
#define INC_TMC6100_H_

#include <project.h>

#define TMC6100_GCONF          0x00
#define TMC6100_GSTAT          0x01
#define TMC6100_IOIN_OUTPUT    0x04
#define TMC6100_OTP_PROG       0x06
#define TMC6100_OTP_READ       0x07
#define TMC6100_FACTORY_CONF   0x08
#define TMC6100_SHORT_CONF     0x09
#define TMC6100_DRV_CONF       0x0A

#define DISABLE 	0
#define SINGLELINE 	1
#define FAULTDIRECT 2
#define NORMAL_OP 	6
#define TEST_MODE 	7

//GSTAT register bits
#define OVERTEMPERATURE 2
#define UNDERVOLTAGE 	3
#define SHORT_U			4
#define SHORT_V 		8
#define SHORT_W 		12

#define DRVSTR_WEAK 	0b00000000
#define DRVSTR_WEAKTC 	0b00000100
#define DRVSTR_MED 		0b00001000
#define DRVSTR_STRONG 	0b00001100

#define READ_MASK 	0x00
#define WRITE_MASK 	0x80

typedef enum{
	NO_FAULT = 0,
	UNKNOWN = 1,
	OVER_TEMPERATURE = 2,
	UNDER_VOLTAGE = 3,
	SHORT_CIRCUIT = 4
}TMCFaults;

TMCFaults checkFaults();
bool initializeTMC6100();
uint32_t read_register_tmc6100(uint8_t address);


#endif /* INC_TMC6100_H_ */
