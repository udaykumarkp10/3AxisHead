/*
 * tmc4671.h
 *
 *  Created on: May 17, 2024
 *      Author: Rajdeep
 */

#ifndef INC_TMC4671_H_
#define INC_TMC4671_H_

#include <project.h>
#include "br24l64f.h"
#include "tmc6100.h"
#include "pcap.h"

typedef struct{
	uint32_t encoder_resolution;
	uint32_t encoder_direction;
	int32_t encoder_zero_offset;
	int32_t soft_limit_positive;
	int32_t soft_limit_negative;
	uint32_t position_error_limit;

	uint32_t current_centre_value;
	uint32_t current_limit_homing;
	uint32_t current_limit_servo;
	uint32_t voltage_limit_homing;
	uint32_t velocity_limit_servo;
	uint32_t torque_limit_servo;
	uint32_t acceleration_limit_servo;

	uint32_t current_P_gain;
	uint32_t current_I_gain;
	uint32_t velocity_P_gain;
	uint32_t velocity_I_gain;
	uint32_t position_P_gain;
	uint32_t position_I_gain;
}Parameters;

typedef enum{
	NO_LIMIT = 0,
	SOFT_NEGATIVE = 1,
	SOFT_POSITIVE = 2,
	HARD_NEGATIVE = 3,
	HARD_POSITIVE = 4
}TMCLimits;

typedef struct{
	bool servo_ready;
	bool servo_enable;
	bool homing_done;
	bool target_reached;
	TMCLimits limits;
	TMCFaults faults;
	bool wrong_command;
}TMCStatusFlags;

typedef struct{
	int32_t current_position;
	int32_t target_position;
	int32_t current_velocity;
	int32_t target_velocity;
	uint32_t motor_current;
	Parameters tmc_parameters;
	TMCStatusFlags tmc_flags;
	Velocity_Profile speed_profile;
}TMC4671_Controller;

void setEncoderResolution(TMC4671_Controller *tmc4671_controller, uint32_t resolution_nanometers);
uint32_t getEncoderResolution(TMC4671_Controller *tmc4671_controller);

void setEncoderDirection(TMC4671_Controller *tmc4671_controller, bool direction);
bool getEncoderDirection(TMC4671_Controller *tmc4671_controller);

void setZeroOffset(TMC4671_Controller *tmc4671_controller, int32_t offset_microns);
int32_t getZeroOffset(TMC4671_Controller *tmc4671_controller);

uint32_t getCoilCurrent(TMC4671_Controller *tmc4671_controller);

void setCurrentLimitHoming(TMC4671_Controller *tmc4671_controller, uint32_t current_limit);
uint32_t getCurrentLimitHoming(TMC4671_Controller *tmc4671_controller);

void setCurrentLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t current_limit);
uint32_t getCurrentLimitServo(TMC4671_Controller *tmc4671_controller);

void setVoltageLimitHoming(TMC4671_Controller *tmc4671_controller, uint32_t voltage_limit);
uint32_t getVoltageLimitHoming(TMC4671_Controller *tmc4671_controller);

void setVelocityLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t velocity_limit);
uint32_t getVelocityLimitServo(TMC4671_Controller *tmc4671_controller);

void setAccelerationLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t acceleration_limit);
uint32_t getAccelerationLimitServo(TMC4671_Controller *tmc4671_controller);

void setTorqueLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t torque_limit);
uint32_t getTorqueLimitServo(TMC4671_Controller *tmc4671_controller);

void setSoftPositiveLimit(TMC4671_Controller *tmc4671_controller, int32_t limit_microns);
int32_t getSoftPositiveLimit(TMC4671_Controller *tmc4671_controller);

void setSoftNegativeLimit(TMC4671_Controller *tmc4671_controller, int32_t limit_microns);
int32_t getSoftNegativeLimit(TMC4671_Controller *tmc4671_controller);

void setAbsoluteTargetPosition(TMC4671_Controller *tmc4671_controller, int32_t target_microns);
void setIncrementalTargetPosition(TMC4671_Controller *tmc4671_controller, int32_t target_microns);
int32_t getTargetPosition(TMC4671_Controller *tmc4671_controller);

int32_t getActualPosition(TMC4671_Controller *tmc4671_controller);

void setTargetVelocity(TMC4671_Controller *tmc4671_controller, int32_t target_microns_per_second);
int32_t getTargetVelocity(TMC4671_Controller *tmc4671_controller);

int32_t getActualVelocity(TMC4671_Controller *tmc4671_controller);

void setMaxPositionError(TMC4671_Controller *tmc4671_controller, uint32_t error_microns);
uint32_t getMaxPositionError(TMC4671_Controller *tmc4671_controller);

void setCurrentGainP(TMC4671_Controller *tmc4671_controller, uint32_t gain);
uint32_t getCurrentGainP(TMC4671_Controller *tmc4671_controller);

void setCurrentGainI(TMC4671_Controller *tmc4671_controller, uint32_t gain);
uint32_t getCurrentGainI(TMC4671_Controller *tmc4671_controller);

void setVelocityGainP(TMC4671_Controller *tmc4671_controller, uint32_t gain);
uint32_t getVelocityGainP(TMC4671_Controller *tmc4671_controller);

void setVelocityGainI(TMC4671_Controller *tmc4671_controller, uint32_t gain);
uint32_t getVelocityGainI(TMC4671_Controller *tmc4671_controller);

void setPositionGainP(TMC4671_Controller *tmc4671_controller, uint32_t gain);
uint32_t getPositionGainP(TMC4671_Controller *tmc4671_controller);

void setPositionGainI(TMC4671_Controller *tmc4671_controller, uint32_t gain);
uint32_t getPositionGainI(TMC4671_Controller *tmc4671_controller);

TMCStatusFlags getEventStatus(TMC4671_Controller *tmc4671_controller);
uint16_t getEventStatusWord_TMC(TMC4671_Controller *tmc4671_controller);

void startHoming(TMC4671_Controller *tmc4671_controller);
void servoEnable(TMC4671_Controller *tmc4671_controller, bool enable);
void servoRun(TMC4671_Controller *tmc4671_controller);
void clearFaults(TMC4671_Controller *tmc4671_controller);
void stopMovement(TMC4671_Controller *tmc4671_controller);
void saveParameters(TMC4671_Controller *tmc4671_controller);
void loadDefaultParameters(TMC4671_Controller *tmc4671_controller);

bool initializeTMC4671(TMC4671_Controller *tmc4671_controller);
uint32_t getFirmwareVersion();
void setWrongCommandFlag(TMC4671_Controller *tmc4671_controller, bool is_wrong);

//-------------internal functions, not to be called-----------------------------------
void loadParameters(TMC4671_Controller *tmc4671_controller);
bool is_tmc4671_connected();
uint32_t read_register_tmc4671(uint8_t address);
bool write_register_tmc4671(uint8_t address, uint32_t data);
bool setup_controller(TMC4671_Controller *tmc4671_controller);
bool setup_PID(TMC4671_Controller *tmc4671_controller);
bool set_velocity(int32_t velocity);
bool set_position(int32_t position);
int32_t get_velocity();
int32_t get_position();
int32_t get_position_error();
uint32_t get_motor_current();
void control_enable();
void control_disable();
void saveDefaultParameters();
void makeDefaultParametersCurrent();

//--------------REGISTER ADDRESSES-----------------------------------
#define READ_MASK 0x00
#define WRITE_MASK 0x80

#define TMC4671_CHIPINFO_DATA                      0x00
#define TMC4671_CHIPINFO_ADDR                      0x01

#define TMC4671_ADC_RAW_DATA                       0x02
#define TMC4671_ADC_RAW_ADDR                       0x03

#define TMC4671_dsADC_MCFG_B_MCFG_A                0x04
#define TMC4671_dsADC_MCLK_A                       0x05
#define TMC4671_dsADC_MCLK_B                       0x06
#define TMC4671_dsADC_MDEC_B_MDEC_A                0x07

#define TMC4671_ADC_I1_SCALE_OFFSET                0x08
#define TMC4671_ADC_I0_SCALE_OFFSET                0x09
#define TMC4671_ADC_I_SELECT                       0x0A
#define TMC4671_ADC_I1_I0_EXT                      0x0B

#define TMC4671_DS_ANALOG_INPUT_STAGE_CFG          0x0C

#define TMC4671_AENC_0_SCALE_OFFSET                0x0D
#define TMC4671_AENC_1_SCALE_OFFSET                0x0E
#define TMC4671_AENC_2_SCALE_OFFSET                0x0F

#define TMC4671_AENC_SELECT                        0x11

#define TMC4671_ADC_IWY_IUX                        0x12
#define TMC4671_ADC_IV                             0x13
#define TMC4671_AENC_WY_UX                         0x15
#define TMC4671_AENC_VN                            0x16

#define TMC4671_PWM_POLARITIES                     0x17
#define TMC4671_PWM_MAXCNT                         0x18
#define TMC4671_PWM_BBM_H_BBM_L                    0x19
#define TMC4671_PWM_SV_CHOP                        0x1A
#define TMC4671_MOTOR_TYPE_N_POLE_PAIRS            0x1B

#define TMC4671_PHI_E_EXT                          0x1C
#define TMC4671_PHI_M_EXT                          0x1D
#define TMC4671_POSITION_EXT                       0x1E
#define TMC4671_OPENLOOP_MODE                      0x1F
#define TMC4671_OPENLOOP_ACCELERATION              0x20
#define TMC4671_OPENLOOP_VELOCITY_TARGET           0x21
#define TMC4671_OPENLOOP_VELOCITY_ACTUAL           0x22
#define TMC4671_OPENLOOP_PHI                       0x23
#define TMC4671_UQ_UD_EXT                          0x24

#define TMC4671_ABN_DECODER_MODE                   0x25
#define TMC4671_ABN_DECODER_PPR                    0x26
#define TMC4671_ABN_DECODER_COUNT                  0x27
#define TMC4671_ABN_DECODER_COUNT_N                0x28
#define TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET     0x29
#define TMC4671_ABN_DECODER_PHI_E_PHI_M            0x2A

#define TMC4671_ABN_2_DECODER_MODE                 0x2C
#define TMC4671_ABN_2_DECODER_PPR                  0x2D
#define TMC4671_ABN_2_DECODER_COUNT                0x2E
#define TMC4671_ABN_2_DECODER_COUNT_N              0x2F
#define TMC4671_ABN_2_DECODER_PHI_M_OFFSET         0x30
#define TMC4671_ABN_2_DECODER_PHI_M                0x31

#define TMC4671_HALL_MODE                          0x33
#define TMC4671_HALL_POSITION_060_000              0x34
#define TMC4671_HALL_POSITION_180_120              0x35
#define TMC4671_HALL_POSITION_300_240              0x36
#define TMC4671_HALL_PHI_E_PHI_M_OFFSET            0x37
#define TMC4671_HALL_DPHI_MAX                      0x38
#define TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E      0x39
#define TMC4671_HALL_PHI_M                         0x3A

#define TMC4671_AENC_DECODER_MODE                  0x3B
#define TMC4671_AENC_DECODER_N_MASK_N_THRESHOLD    0x3C
#define TMC4671_AENC_DECODER_PHI_A_RAW             0x3D
#define TMC4671_AENC_DECODER_PHI_A_OFFSET          0x3E
#define TMC4671_AENC_DECODER_PHI_A                 0x3F

#define TMC4671_AENC_DECODER_PPR                   0x40
#define TMC4671_AENC_DECODER_COUNT                 0x41
#define TMC4671_AENC_DECODER_COUNT_N               0x42
#define TMC4671_AENC_DECODER_PHI_E_PHI_M_OFFSET    0x45
#define TMC4671_AENC_DECODER_PHI_E_PHI_M           0x46
#define TMC4671_AENC_DECODER_POSITION              0x47

#define TMC4671_CONFIG_DATA                        0x4D
#define TMC4671_CONFIG_ADDR                        0x4E

#define TMC4671_VELOCITY_SELECTION                 0x50
#define TMC4671_POSITION_SELECTION                 0x51
#define TMC4671_PHI_E_SELECTION                    0x52
#define TMC4671_PHI_E                              0x53

#define TMC4671_PID_FLUX_P_FLUX_I                  0x54
#define TMC4671_PID_TORQUE_P_TORQUE_I              0x56
#define TMC4671_PID_VELOCITY_P_VELOCITY_I          0x58
#define TMC4671_PID_POSITION_P_POSITION_I          0x5A
#define TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS  0x5C
#define TMC4671_PIDOUT_UQ_UD_LIMITS                0x5D
#define TMC4671_PID_TORQUE_FLUX_LIMITS             0x5E
#define TMC4671_PID_VELOCITY_LIMIT                 0x60
#define TMC4671_PID_POSITION_LIMIT_LOW             0x61
#define TMC4671_PID_POSITION_LIMIT_HIGH            0x62

#define TMC4671_MODE_RAMP_MODE_MOTION              0x63
#define TMC4671_PID_TORQUE_FLUX_TARGET             0x64
#define TMC4671_PID_TORQUE_FLUX_OFFSET             0x65
#define TMC4671_PID_VELOCITY_TARGET                0x66
#define TMC4671_PID_VELOCITY_OFFSET                0x67
#define TMC4671_PID_POSITION_TARGET                0x68

#define TMC4671_PID_TORQUE_FLUX_ACTUAL             0x69
#define TMC4671_PID_VELOCITY_ACTUAL                0x6A
#define TMC4671_PID_POSITION_ACTUAL                0x6B

#define TMC4671_PID_ERROR_DATA                     0x6C
#define TMC4671_PID_ERROR_ADDR                     0x6D
#define TMC4671_INTERIM_DATA                       0x6E
#define TMC4671_INTERIM_ADDR                       0x6F

#define TMC4671_WATCHDOG_CFG                       0x74
#define TMC4671_ADC_VM_LIMITS                      0x75
#define TMC4671_INPUTS_RAW                         0x76
#define TMC4671_OUTPUTS_RAW                        0x77

#define TMC4671_STEP_WIDTH                         0x78

#define TMC4671_UART_BPS                           0x79
#define TMC4671_UART_ADDRS                         0x7A

#define TMC4671_GPIO_dsADCI_CONFIG                 0x7B

#define TMC4671_STATUS_FLAGS                       0x7C
#define TMC4671_STATUS_MASK                        0x7D

//--------------REGISTER VALUES-----------------------------------
// Motor type &  PWM configuration
#define SET_MOTOR_TYPE_N_POLE_PAIRS 		0x00010001
#define SET_PWM_POLARITIES 					0x00000000
#define SET_PWM_MAXCNT 						0x00000F9F
#define SET_PWM_BBM_H_BBM_L 				0x00002828
#define SET_PWM_SV_CHOP 					0x00000007

// ADC configuration
#define SET_ADC_I_SELECT 					0x14000200
#define SET_dsADC_MCFG_B_MCFG_A 			0x00100010
#define SET_dsADC_MCLK_A 					0x20000000
#define SET_dsADC_MCLK_B 					0x20000000
#define SET_dsADC_MDEC_B_MDEC_A 			0x052C052C
#define SET_ADC_I0_SCALE_OFFSET 			0x01318189		//scale-305, offset - 33161
#define SET_ADC_I1_SCALE_OFFSET 			0x01000000

// ABN encoder #define SETtings
#define SET_ABN_DECODER_MODE 				0x00000100		//enable cln bit to auto load COUNT_N => COUNT on index pulse //0x00001100 {might need to disable this after homing done}
#define SET_ABN_DECODER_COUNT 				0x00000000
#define SET_ABN_DECODER_COUNT_N 			0x00000000
#define SET_ABN_DECODER_PHI_E_PHI_M_OFFSET 	0x00000000
#define TOTAL_MAPPED_TRAVEL_MICRONS			20000.0
#define TOTAL_DECODER_REGISTER_COUNT		65536.0
#define STOP_TOLERANCE_DECODER_COUNT		1638			//500 microns
#define SET_ABN_DECODER_MODE_DIS 			0x00000000

//selectors
#define SET_PHI_E_SELECTION_ABN				0x00000003		//phi_e_abn
#define SET_PHI_E_SELECTION_OPENLOOP		0x00000002		//phi_e_openloop
#define SET_VELOCITY_SELECTION 				0x00000009		//phi_m_abn and default
#define SET_POSITION_SELECTION 				0x00000009		//phi_m_abn
#define SET_ADC_I_SELECT 					0x14000200		//I0_raw, I0_ext, I0, I1, I1
#define SET_PHI_E_SELECTION_EXT				0x00000001		//phi_e_ext

//limits
#define SET_PIDOUT_UQ_UD_LIMITS 			0x00007FFF		//32767

//initialization
#define SET_PID_POSITION_ACTUAL_ZERO 		0x00000000

#define SET_MOTION_MODE_VELOCITY 			0x00000002		//velocity_mode
#define SET_MOTION_MODE_POSITION 			0x00000003		//position_mode
#define SET_MOTION_MODE_UQ_UD 				0x00000008		//uq_ud_mode
#define SET_MOTION_MODE_STOPPED 			0x00000000		//stopped_mode
#define SET_MOTION_MODE_TORQUE 				0x00000001		//torque_mode
#define SET_PHI_E_EXT						0x00000000

//EEPROM-addresses-factory-values

#define DEF_ADDR_ENCODER_RESOLUTION			0x0000
#define DEF_ADDR_ENCODER_DIRECTION			0x0008
#define DEF_ADDR_ENCODER_ZERO_OFFSET		0x0010
#define DEF_ADDR_SOFT_LIMIT_POSITIVE		0x0018
#define DEF_ADDR_SOFT_LIMIT_NEGATIVE		0x0020
#define DEF_ADDR_POSITION_ERROR_LIMIT		0X0028

#define DEF_ADDR_CURRENT_CENTER_VAL			0x0030
#define DEF_ADDR_CURRENT_LIMIT_HOMING		0x0038
#define DEF_ADDR_CURRENT_LIMIT_SERVO		0x0040
#define DEF_ADDR_VOLTAGE_LIMIT_HOMING		0x0048
#define DEF_ADDR_VELOCITY_LIMIT_SERVO		0x0050
#define DEF_ADDR_TORQUE_LIMIT_SERVO			0x0058

#define DEF_ADDR_CURRENT_P_GAIN				0x0060
#define DEF_ADDR_CURRENT_I_GAIN				0x0068
#define DEF_ADDR_VELOCITY_P_GAIN			0x0070
#define DEF_ADDR_VELOCITY_I_GAIN			0x0078
#define DEF_ADDR_POSITION_P_GAIN			0x0080
#define DEF_ADDR_POSITION_I_GAIN			0x0088

#define DEF_ADDR_ACCEL_LIMIT_SERVO			0x0090

#define DEF_ADDR_VERSION_NUMBER				0x0288

//EEPROM-addresses-saved-values

#define SAVE_ADDR_ENCODER_RESOLUTION		0x0100
#define SAVE_ADDR_ENCODER_DIRECTION			0x0108
#define SAVE_ADDR_ENCODER_ZERO_OFFSET		0x0110
#define SAVE_ADDR_SOFT_LIMIT_POSITIVE		0x0118
#define SAVE_ADDR_SOFT_LIMIT_NEGATIVE		0x0120
#define SAVE_ADDR_POSITION_ERROR_LIMIT		0X0128

#define SAVE_ADDR_CURRENT_CENTER_VAL		0x0130
#define SAVE_ADDR_CURRENT_LIMIT_HOMING		0x0138
#define SAVE_ADDR_CURRENT_LIMIT_SERVO		0x0140
#define SAVE_ADDR_VOLTAGE_LIMIT_HOMING		0x0148
#define SAVE_ADDR_VELOCITY_LIMIT_SERVO		0x0150
#define SAVE_ADDR_TORQUE_LIMIT_SERVO		0x0158

#define SAVE_ADDR_CURRENT_P_GAIN			0x0160
#define SAVE_ADDR_CURRENT_I_GAIN			0x0168
#define SAVE_ADDR_VELOCITY_P_GAIN			0x0170
#define SAVE_ADDR_VELOCITY_I_GAIN			0x0178
#define SAVE_ADDR_POSITION_P_GAIN			0x0180
#define SAVE_ADDR_POSITION_I_GAIN			0x0188

#define SAVE_ADDR_ACCEL_LIMIT_SERVO			0x0190

//-----------------factory default values-----------------------------------------------------------

#define DEFAULT_ENCODER_RESOLUTION		10000	//2x: 10000		5x: 1000
#define DEFAULT_ENCODER_DIRECTION		1		//2x: 1 		5x: 1
#define DEFAULT_ENCODER_ZERO_OFFSET		0		//2x: 0		 	5x: 0
#define DEFAULT_SOFT_LIMIT_POSITIVE		16500	//2x: 16500 	5x: 16500
#define DEFAULT_SOFT_LIMIT_NEGATIVE		-16500	//2x: -16500	5x: -16500
#define DEFAULT_POSITION_ERROR_LIMIT	33		//2x: 33		5x: 164

#define DEFAULT_CURRENT_CENTER_VAL		32990	//2x: 32990		5x: 33161
#define DEFAULT_CURRENT_LIMIT_HOMING	1500	//2x: 1500		5x: 1500
#define DEFAULT_CURRENT_LIMIT_SERVO		4000	//2x: 4000		5x: 4000
#define DEFAULT_VOLTAGE_LIMIT_HOMING	5000	//2x: 5000		5x: 5000
#define DEFAULT_VELOCITY_LIMIT_SERVO	40		//2x: 40		5x: 30
#define DEFAULT_TORQUE_LIMIT_SERVO		10000	//2x: 10000		5x: 8000
#define DEFAULT_VELOCITY_STARTUP_SERVO	5		//2x: 5			5x: 5
#define DEFAULT_ACCEL_LIMIT_SERVO		1		//2x: 1			5x: 1

#define DEFAULT_CURRENT_P_GAIN			1500	//2x: 1500		5x: 1500
#define DEFAULT_CURRENT_I_GAIN			6000	//2x: 6000		5x: 15000
#define DEFAULT_VELOCITY_P_GAIN			800		//2x: 800		5x: 3000
#define DEFAULT_VELOCITY_I_GAIN			2000	//2x: 2000		5x: 16000
#define DEFAULT_POSITION_P_GAIN			50		//2x: 50		5x: 120
#define DEFAULT_POSITION_I_GAIN			0		//2x: 0			5x: 0

#define FIRMWARE_VERSION_NUMBER			202410

//-----------------PID error data------------------------------------------------------------------
#define PID_TORQUE_ERROR 			0
#define PID_FLUX_ERROR 				1
#define PID_VELOCITY_ERROR 			2
#define PID_POSITION_ERROR 			3
#define PID_TORQUE_ERROR_SUM 		4
#define PID_FLUX_ERROR_SUM 			5
#define PID_VELOCITY_ERROR_SUM 		6
#define PID_POSITION_ERROR_SUM 		7

#endif /* INC_TMC4671_H_ */
