/*
 * pcap.c
 *
 *  Created on: May 22, 2024
 *      Author: LAB
 *      MODIFICATION: spikes removed
 */


#include "pcap.h"

MovingAverage mAvg; // Define the moving average variable

uint8_t SUM_OF_PCAP = 0;

uint32_t pcap_capval;
uint32_t pcap_tempval;
uint32_t pcap_capval_check1;
uint32_t pcap_capval_check2;
uint32_t pcap_capval_avg;

// init / check the pcap interface on I2C, return true if init ok
bool pcap_init(){
	uint8_t buf[10] = {0};
	HAL_StatusTypeDef ret;
    // read and check TEST data 1 byte
	buf[0] = PCAP_TEST_ADDR;
	ret = HAL_I2C_Master_Transmit(&PCAP_PORT, PCAP_ADDR, buf, 1, PCAP_MAX_DELAY);
	if(ret != HAL_OK){
		printf("Error sending Pcap test command, code = %d\n", ret);
		return false;
	}else{
		ret = HAL_I2C_Master_Receive(&PCAP_PORT, PCAP_ADDR, buf, 1, PCAP_MAX_DELAY);
		if(ret != HAL_OK){
			printf("Bad response while reading Pcap test command, bytes received = %d\n", ret);
			return false;
		}else{
			if(buf[0] != PCAP_TEST_DATA){
				printf("Bad response received from Pcap test command, data received = %x\n", buf[0]);
				return false;
			}
		}
	}
	initMovingAverage(&mAvg);


	printf("Pcap test command succeeded\n");
	return true;
}

// one pcap scan - get sts, cap, temp values
PcapErrorStatus pcap_scan(){
	uint8_t buf[10] = {0};
	HAL_StatusTypeDef ret;
	PcapErrorStatus errorStatus = PCAP_NO_ERROR;

	// read and display error status 2 bytes
	buf[0] = PCAP_STS_ADDR;
	ret = HAL_I2C_Master_Transmit(&PCAP_PORT, PCAP_ADDR, buf, 1, PCAP_MAX_DELAY);
	if(ret != HAL_OK){
		printf("Error sending RD STS command, code = %d\n", ret);
		errorStatus |= PCAP_COMM_ERROR;
		return errorStatus;
	}else{
		ret = HAL_I2C_Master_Receive(&PCAP_PORT, PCAP_ADDR, buf, 2, PCAP_MAX_DELAY);
		if(ret != HAL_OK){
			printf("Bad response while reading RD STS command, bytes received = %d\n", ret);
			errorStatus |= PCAP_COMM_ERROR;
			return errorStatus;
		}else{
			uint16_t pcap_sts = ((uint16_t)buf[1] << 8) | (uint16_t)buf[0];
			errorStatus = PCAP_NO_ERROR;
			printf("Status = %x\n", pcap_sts);
		}
	}

	// read and display cap value 4 bytes
	buf[0] = PCAP_CAP_ADDR;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(&PCAP_PORT, PCAP_ADDR, buf, 1, PCAP_MAX_DELAY);
	if(ret != HAL_OK){
		printf("Error sending RD CAP command, code = %d\n", ret);
		Etc_Buffer_In.LANByte[5] |= PCAP_COMM_ERROR;
		return errorStatus;
	}else{
		ret = HAL_I2C_Master_Receive(&PCAP_PORT, PCAP_ADDR, buf, 4, PCAP_MAX_DELAY);
		if(ret != HAL_OK){
			printf("Bad response while reading RD CAP command, bytes received = %d\n", ret);
			errorStatus |= PCAP_COMM_ERROR;
			return errorStatus;
		}else{
			pcap_capval_check1 = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];

		   delay_us(DEBOUNCE_TIME_US);

			buf[0] = PCAP_CAP_ADDR;
			buf[1] = 0;
			ret = HAL_I2C_Master_Transmit(&PCAP_PORT, PCAP_ADDR, buf, 1, PCAP_MAX_DELAY);
			if(ret != HAL_OK){
				printf("Error sending RD CAP command, code = %d\n", ret);
				errorStatus |= PCAP_COMM_ERROR;
				return errorStatus;
			}else{
				ret = HAL_I2C_Master_Receive(&PCAP_PORT, PCAP_ADDR, buf, 4, PCAP_MAX_DELAY);
				if(ret != HAL_OK){
					printf("Bad response while reading RD CAP command, bytes received = %d\n", ret);
					errorStatus |= PCAP_COMM_ERROR;
					return errorStatus;
				}else{
					pcap_capval_check2 = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];

					if(abs(pcap_capval_check1 - pcap_capval_check2) < DEBOUNCE_CAP_TOLERANCE){
						pcap_capval = pcap_capval_check2;

						if (pcap_capval > DEBOUNCE_CAP_TOLERANCE){
							Etc_Buffer_In.LANLong[2] = pcap_capval;
						}
						else {
							updateMovingAverage(&mAvg, pcap_capval);
							pcap_capval_avg = mAvg.out;

							//Etc_Buffer_In.LANLong[6] = pcap_capval;
							Etc_Buffer_In.LANLong[2] = pcap_capval_avg;

							if (pcap_capval_avg == PCAP_TIP_TOUCH) {
								errorStatus |= PCAP_TIP_TOUCH_ERROR;
							}
							printf("Filtered Capacitive Value = %lu\n", pcap_capval_avg);
						}
					}
				}
			}
		}

	}

	//read and display temperature value 4 bytes
	buf[0] = PCAP_TEMP_ADDR;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	ret = HAL_I2C_Master_Transmit(&PCAP_PORT, PCAP_ADDR, buf, 1, PCAP_MAX_DELAY);
	if(ret != HAL_OK){
		printf("Error sending RD CAP command, code = %d\n", ret);
		errorStatus |= PCAP_COMM_ERROR;
		return errorStatus;
	}else{
		ret = HAL_I2C_Master_Receive(&PCAP_PORT, PCAP_ADDR, buf, 4, PCAP_MAX_DELAY);
		if(ret != HAL_OK){
			printf("Bad response while reading RD CAP command, bytes received = %d\n", ret);
			errorStatus |= PCAP_COMM_ERROR;
			return errorStatus;
		}else{
			pcap_tempval = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
			Etc_Buffer_In.LANLong[3] = pcap_tempval;
			// printf("Temperature Value = %lu\n", pcap_tempval);
		}
	}
	return errorStatus;
}

uint16_t getEventStatusWord_PCAP(PcapErrorStatus *pcap_status) {
    uint16_t status_word = 0;

    switch (*pcap_status) {
        case PCAP_NO_ERROR:
            break;
        case PCAP_COMM_ERROR:
            status_word |= 0x0001 << 13;
            break;
        case PCAP_TIP_TOUCH_ERROR:
            status_word |= 0x0001 << 14;
            break;
    }

    return status_word;
}


//This function is used to create delay in micro second.
void delay_us(uint16_t us) {
	HAL_TIM_Base_Start(&PCAP_TIMER);

	__HAL_TIM_SET_COUNTER(&PCAP_TIMER, 0);  // set the counter value a 0

	while (__HAL_TIM_GET_COUNTER(&PCAP_TIMER) < us);  // wait for the counter to reach the us input in the parameter

	HAL_TIM_Base_Stop(&PCAP_TIMER);
}
