/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc4671.h"
#include "adxl345.h"
#include "lan9252.h"
#include "pcap.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
bool pcap_init_ok;
bool etc_init_ok;
bool adxl345_init_ok;

// TMC4671 variables
volatile bool index_found = false;
TMC4671_Controller tmc4671_controller;
bool no_error_comm = true;
bool no_error_drive = false;
bool data_received = false;

uint16_t TxCommand = 0;


// EtherCAt variables
bool set_command_flag = false;
bool get_command_flag = false;
bool continuous_tx_flag = false;
bool accelerometer_flag = false;
bool command_processed = false;

uint16_t etc_old_command = 0;
int32_t etc_old_data = 0;
uint16_t etc_new_command = 0;
int32_t etc_new_data = 0;

uint32_t etc_digital_output;

uint32_t etc_analog_output_0_1;

uint32_t etc_analog_output_2_3;

// Define four new variables for storing the 16-bit split outputs
uint16_t etc_analog_output_0;
uint16_t etc_analog_output_1;
uint16_t etc_analog_output_2;
uint16_t etc_analog_output_3;

// Send variables to EtherCAt
int32_t TxData = 0;
uint16_t TxStatus = 0;

// Pcap variable of structure
PcapErrorStatus Pcap_status;

//test

uint8_t set_Execution_count = 0;
uint8_t get_Execution_count = 0;

uint8_t set_sent_count = 0;
uint8_t get_sent_count = 0;
uint8_t adxl_sent_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI5_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ENC_INDEX_Pin){
    	if(!index_found){
    		index_found = true;
    	}
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  etc_init_ok = etc_init();
  pcap_init_ok = pcap_init();
  adxl345_init_ok = Adxl345_init();

  if(initializeTMC4671(&tmc4671_controller)){
	  no_error_drive = true;
  }else{
	  no_error_drive = false;
  }

  HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*-------------------------- GET DATA FROM PCAP AND ETHERCAT----------------------------------------*/

	  if (pcap_init_ok) {
		  Pcap_status = pcap_scan();
	  } else {
		  pcap_init_ok = pcap_init();
		  Pcap_status = PCAP_COMM_ERROR;
	  }

	  if (etc_init_ok) {
		  etc_scan();
	  } else {
		  etc_init_ok = etc_init();
	  }

	  /*-------------------------- RECEIVED DATA FROM ETHERCAT ----------------------------------------*/

	  etc_new_command = (uint16_t)Etc_Buffer_Out.LANLong[0];
	  etc_new_data =  (int32_t) Etc_Buffer_Out.LANLong[1];

	  if (((etc_new_command >= 0) && (etc_new_command <= 21)) || ((etc_new_command >= 100) && (etc_new_command <= 105))) {
	      set_command_flag = true;
	  } else {
		  set_command_flag = false;
	  }

	  if ((etc_new_command >= 50 && etc_new_command <= 73) || (etc_new_command == 200)) {
		  get_command_flag = true;
	  } else {
		  get_command_flag = false;
	  }

	  if ((etc_new_command == 301 || etc_new_command == 302 || etc_new_command == 303)) {
		  accelerometer_flag = true;
	  } else {
		  accelerometer_flag = false;
	  }

	  etc_digital_output = Etc_Buffer_Out.LANLong[5];
	  etc_analog_output_0_1 = Etc_Buffer_Out.LANLong[6];
	  etc_analog_output_2_3 = Etc_Buffer_Out.LANLong[7];

	  // Split the 32-bit floats into two 16-bit outputs each
	  etc_analog_output_0 = (uint16_t)(((uint32_t)etc_analog_output_0_1) & 0xFFFF);
	  etc_analog_output_1 = (uint16_t)((((uint32_t)etc_analog_output_0_1) >> 16) & 0xFFFF);
	  etc_analog_output_2 = (uint16_t)(((uint32_t)etc_analog_output_2_3) & 0xFFFF);
	  etc_analog_output_3 = (uint16_t)((((uint32_t)etc_analog_output_2_3) >> 16) & 0xFFFF);

	  /*---------------------------PROCESS REECIVED COMMAND AND DATA-----------------------------------------*/

	  if (set_command_flag) {
		  if ((etc_old_command != etc_new_command) || (etc_old_data != etc_new_data)) {
				switch (etc_new_command) {
				case 0:
					TxData = etc_new_data;
					continuous_tx_flag = false;
					command_processed = true;
					break;

				case 1:
					TxData = etc_new_data;
					set_Execution_count++;
					if ((etc_new_data <= 0) || (etc_new_data > 1000000)) {
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setEncoderResolution(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 2:
					TxData = etc_new_data;
					set_Execution_count++;
					if ((etc_new_data != 0) && (etc_new_data != 1)) {//cannot be anything other than 0 or 1
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setEncoderDirection(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 3:
					TxData = etc_new_data;
					if (( etc_new_data <= -5000) || ( etc_new_data >= 5000)) {//cannot be less than -5000um = -5mm or greater than 5000um = 5mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setZeroOffset(&tmc4671_controller, (int32_t) etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 4:
					TxData = etc_new_data;
					if (( etc_new_data <= 0) || ( etc_new_data > 12000)) {//cannot be 0, negative number or more than 12000um = 12mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setSoftPositiveLimit(&tmc4671_controller, (int32_t) etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 5:
					TxData = etc_new_data;
					if (( etc_new_data >= 0) || ( etc_new_data < -12000)) {	//cannot be 0, positive number or less than -12000um = -12mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setSoftNegativeLimit(&tmc4671_controller, (int32_t) etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 6:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 2000)) {	//cannot be 0, negative number or more than 2000um = 2mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setMaxPositionError(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 7:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 5000)) {	//cannot be 0, negative number or more than 5000
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setCurrentLimitHoming(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 8:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 15000)) {//cannot be 0, negative number or more than 15000
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setCurrentLimitServo(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 9:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 8000)) {	//cannot be 0, negative number or more than 8000
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setVoltageLimitHoming(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 10:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 500)) {//cannot be 0, negative number or more than 500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setVelocityLimitServo(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 11:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 30000)) {//cannot be 0, negative number or more than 30000
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setTorqueLimitServo(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 12:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 32500)) {//cannot be 0, negative number or more than 32500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setCurrentGainP(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 13:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 32500)) {//cannot be 0, negative number or more than 32500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setCurrentGainI(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 14:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 32500)) {//cannot be 0, negative number or more than 32500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setVelocityGainP(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					continuous_tx_flag = false;
					break;

				case 15:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 32500)) {//cannot be 0, negative number or more than 32500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setVelocityGainI(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 16:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 32500)) {//cannot be 0, negative number or more than 32500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setPositionGainP(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 17:
					TxData = etc_new_data;
					if ((etc_new_data <= 0) || (etc_new_data > 32500)) {//cannot be 0, negative number or more than 32500
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setPositionGainI(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 18:
					TxData = etc_new_data;
					if (( etc_new_data < -12000) || ( etc_new_data > 12000)) {//cannot be less than -12000um = -12mm or greater than 12000um = 12mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setAbsoluteTargetPosition(&tmc4671_controller, (int32_t) etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 19:
					TxData = etc_new_data;
					if (( etc_new_data < -12000) || ( etc_new_data > 12000)) {//cannot be less than -12000um = -12mm or greater than 12000um = 12mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setIncrementalTargetPosition(&tmc4671_controller, (int32_t) etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 20:
					TxData = etc_new_data;
					if ((etc_new_data < -12) || (etc_new_data > 12)) {//cannot be less than -12000um = -12mm or greater than 12000um = 12mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setWrongCommandFlag(&tmc4671_controller, false);
						setTargetVelocity(&tmc4671_controller, etc_new_data);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 21:
					if ((etc_new_data < 1) || (etc_new_data > 40)) {//cannot be less than -12000um = -12mm or greater than 12000um = 12mm
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						setWrongCommandFlag(&tmc4671_controller, false);
						setAccelerationLimitServo(&tmc4671_controller, etc_new_data);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 100:
					TxData = etc_new_data;
					if ((etc_new_data != 0) && (etc_new_data != 1)) {//cannot be anything other than 0 or 1
						setWrongCommandFlag(&tmc4671_controller, true);
					} else {
						servoEnable(&tmc4671_controller, etc_new_data);
						setWrongCommandFlag(&tmc4671_controller, false);
						command_processed = true;
						continuous_tx_flag = false;
					}
					break;

				case 101:
					TxData = 0;
					startHoming(&tmc4671_controller);
					setWrongCommandFlag(&tmc4671_controller, false);
					continuous_tx_flag = false;
					command_processed = true;
					break;

				case 102:
					TxData = 0;
					clearFaults(&tmc4671_controller);
					setWrongCommandFlag(&tmc4671_controller, false);
					continuous_tx_flag = false;
					command_processed = true;
					break;

				case 103:
					TxData = 0;
					saveParameters(&tmc4671_controller);
					setWrongCommandFlag(&tmc4671_controller, false);
					continuous_tx_flag = false;
					command_processed = true;
					break;

				case 104:
					TxData = 0;
					loadDefaultParameters(&tmc4671_controller);
					setWrongCommandFlag(&tmc4671_controller, false);
					continuous_tx_flag = false;
					command_processed = true;
					break;

				case 105:
					TxData = 0;
					stopMovement(&tmc4671_controller);
					setWrongCommandFlag(&tmc4671_controller, false);
					continuous_tx_flag = false;
					command_processed = true;
					break;

				default:
					break;
				}
				etc_old_command = etc_new_command;
				etc_old_data = etc_new_data;
		  }
	  } else if (get_command_flag) {
		  switch(etc_new_command) {
			case 50:
				TxData = getEncoderResolution(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 51:
				TxData = getEncoderDirection(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 52:
				 TxData = getZeroOffset(&tmc4671_controller);
				 setWrongCommandFlag(&tmc4671_controller, false);
				 continuous_tx_flag = true;
				 break;

			case 53:
				TxData = getSoftPositiveLimit(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;


			case 54:
				TxData =  getSoftNegativeLimit(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 55:
				TxData = getMaxPositionError(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 56:
			    TxData = getCurrentLimitHoming(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 57:
				TxData = getCurrentLimitServo(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 58:
				TxData = getVoltageLimitHoming(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 59:
				TxData = getVelocityLimitServo(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 60:
				TxData = getTorqueLimitServo(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 61:
				TxData = getCurrentGainP(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 62:
				TxData = getCurrentGainI(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 63:
				TxData = getVelocityGainP(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 64:
				TxData = getVelocityGainI(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 65:
				TxData = getPositionGainP(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 66:
				TxData = getPositionGainI(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 67:
				TxData = getTargetPosition(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 68:
				TxData = getActualPosition(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 69:
				TxData = getCoilCurrent(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 70:
				TxData = getFirmwareVersion(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 71:
				TxData = getTargetVelocity(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 72:
				TxData = getActualVelocity(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 73:
				TxData = getAccelerationLimitServo(&tmc4671_controller);
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = true;
				break;

			case 200:
				TxData = 0;
				setWrongCommandFlag(&tmc4671_controller, false);
				continuous_tx_flag = false;
				break;

			default:
				TxData = 0;
				setWrongCommandFlag(&tmc4671_controller, true);
				continuous_tx_flag = false;
				break;
		  }
		  etc_old_command = 0;
		  etc_old_data = 0;
	  } else if (accelerometer_flag) {
		  switch(etc_new_command) {
		  case 301:
			  get_Execution_count++;
			  setWrongCommandFlag(&tmc4671_controller, false);
			  TxData = getAcceleration('X');
			  continuous_tx_flag = true;
			  break;

		  case 302:
			  get_Execution_count++;
			  setWrongCommandFlag(&tmc4671_controller, false);
			  TxData = getAcceleration('Y');
			  continuous_tx_flag = true;
			  break;

		  case 303:
			  setWrongCommandFlag(&tmc4671_controller, false);
			  TxData = getAcceleration('Z');
			  continuous_tx_flag = true;
			  break;
		  }
		  etc_old_command = 0;
		  etc_old_data = 0;
	  }

	  /*---------------------------------SERVO FUNCTIONALITY-----------------------------------------*/

	  servoRun(&tmc4671_controller);
	  uint16_t current_status = getEventStatusWord_TMC(&tmc4671_controller);
	  current_status |= 0b1110000011110000;
	  if (current_status != 0b1110000011110000) {
		  no_error_drive = false;
	  } else {
		  no_error_drive = true;
	  }

	  /*---------------------------TRANSMIT DATA TO ETHERCAT ---------------------------------------------*/

	  uint16_t tmc_status = getEventStatusWord_TMC(&tmc4671_controller);
	  uint16_t pcap_status_word = getEventStatusWord_PCAP(&Pcap_status);

	  TxStatus = tmc_status | pcap_status_word;

	  if (set_command_flag) {
		  if (!continuous_tx_flag && command_processed) {
			  set_sent_count++;
			  Etc_Buffer_In.LANLong[0] = ((uint32_t)TxStatus << 16) | (uint32_t)etc_old_command;
			  Etc_Buffer_In.LANLong[1] = TxData;
			  continuous_tx_flag = true;  // Mark that data has been sent
		  }
		  Etc_Buffer_In.LANLong[0] = ((uint32_t)TxStatus << 16) | (uint32_t)etc_new_command;
	  }

	  if (get_command_flag && etc_new_command !=200 ) {
		  get_sent_count++;
		  Etc_Buffer_In.LANLong[0] = ((uint32_t) TxStatus << 16) | (uint32_t) etc_new_command;
		  Etc_Buffer_In.LANInt[1] = TxData;
	  } else {
		  Etc_Buffer_In.LANLong[0] = ((uint32_t) TxStatus << 16) | (uint32_t) etc_new_command;
		  Etc_Buffer_In.LANInt[1] = TxStatus;
	  }

	  if(accelerometer_flag) {
		  adxl_sent_count++;
		  Etc_Buffer_In.LANLong[0] = ((uint32_t) TxStatus << 16) | (uint32_t) etc_new_command;
		  Etc_Buffer_In.LANLong[1] = (int32_t)TxData;
	  }


	  if (no_error_drive) {
		  HAL_GPIO_WritePin(ERROR_DRIVE_GPIO_Port, ERROR_DRIVE_Pin, GPIO_PIN_SET);
	  } else {
		  HAL_GPIO_WritePin(ERROR_DRIVE_GPIO_Port, ERROR_DRIVE_Pin, GPIO_PIN_RESET);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 96-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65536-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ERROR_DRIVE_GPIO_Port, ERROR_DRIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_TMC4671_Pin|CS_TMC6100_Pin|ADXL345_CS_Pin|LAN9252_CS_Pin
                          |FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ERROR_DRIVE_Pin */
  GPIO_InitStruct.Pin = ERROR_DRIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ERROR_DRIVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_TMC4671_Pin CS_TMC6100_Pin ADXL345_CS_Pin LAN9252_CS_Pin
                           FAULT_LED_Pin */
  GPIO_InitStruct.Pin = CS_TMC4671_Pin|CS_TMC6100_Pin|ADXL345_CS_Pin|LAN9252_CS_Pin
                          |FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_STATUS_Pin */
  GPIO_InitStruct.Pin = TMC_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TMC_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INDEX_Pin */
  GPIO_InitStruct.Pin = ENC_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INDEX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTRL_EN_Pin */
  GPIO_InitStruct.Pin = CTRL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CTRL_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
