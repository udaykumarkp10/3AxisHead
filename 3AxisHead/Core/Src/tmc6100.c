/*
 * tmc6100.c
 *
 *  Created on: May 13, 2024
 *      Author: Rajdeep
 */
#include "tmc6100.h"

TMCFaults checkFaults(){
	uint32_t status = read_register_tmc6100(TMC6100_GSTAT);

	uint32_t check = (status >> OVERTEMPERATURE) & 0x00000001;
	if(check == 0x00000001){
		return OVER_TEMPERATURE;
	}

	check = (status >> UNDERVOLTAGE) & 0x00000001;
	if(check == 0x00000001){
		return UNDER_VOLTAGE;
	}

	check = (status >> SHORT_U) & 0x00000001;
	if(check == 0x00000001){
		return SHORT_CIRCUIT;
	}

	check = (status >> SHORT_V) & 0x00000001;
	if(check == 0x00000001){
		return SHORT_CIRCUIT;
	}

	check = (status >> SHORT_W) & 0x00000001;
	if(check == 0x00000001){
		return SHORT_CIRCUIT;
	}

	return UNKNOWN;
}

bool initializeTMC6100(){
	uint8_t xfrbuf[5];
	HAL_StatusTypeDef ret;

	xfrbuf[0]= WRITE_MASK | TMC6100_GCONF;
	xfrbuf[1]= 0x00;
	xfrbuf[2]= 0x00;
	xfrbuf[3]= 0x00;
	xfrbuf[4]= (0x00 << TEST_MODE) | (0x01 << NORMAL_OP) | (0x01 << FAULTDIRECT) | (0x00 << SINGLELINE) | (0x00 << DISABLE);

	TMC6100_CS_SET();
	ret = HAL_SPI_TransmitReceive(&TMC_PORT, xfrbuf, xfrbuf, 5, HAL_MAX_DELAY);
	TMC6100_CS_RESET();

	if(ret != HAL_OK){
		return false;
	}

	HAL_Delay(100);

	xfrbuf[0]= WRITE_MASK | TMC6100_DRV_CONF;
	xfrbuf[1]= 0x00;
	xfrbuf[2]= DRVSTR_WEAK;
	xfrbuf[3]= 0x00;
	xfrbuf[4]= 0x00;

	TMC6100_CS_SET();
	ret = HAL_SPI_TransmitReceive(&TMC_PORT, xfrbuf, xfrbuf, 5, HAL_MAX_DELAY);
	TMC6100_CS_RESET();

	if(ret != HAL_OK){
		return false;
	}

	return true;
}

uint32_t read_register_tmc6100(uint8_t address){
	uint8_t xfrbuf[5];
	HAL_StatusTypeDef ret;

	xfrbuf[0] = READ_MASK | address;
	xfrbuf[1] = 0x00;
	xfrbuf[2] = 0x00;
	xfrbuf[3] = 0x00;
	xfrbuf[4] = 0x00;

	TMC6100_CS_SET();
	ret = HAL_SPI_TransmitReceive(&TMC_PORT, xfrbuf, xfrbuf, 5, HAL_MAX_DELAY);
	TMC6100_CS_RESET();

	if(ret != HAL_OK){
		return 0xFFFFFFFF;
	}else{
		uint32_t xfrdata = 0;
		xfrdata |= xfrbuf[1] << 24;
		xfrdata |= xfrbuf[2] << 16;
		xfrdata |= xfrbuf[3] << 8;
		xfrdata |= xfrbuf[4];

		return xfrdata;
	}
}

