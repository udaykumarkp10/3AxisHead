/*
 * tmc4671.c
 *
 *  Created on: May 17, 2024
 *      Author: Rajdeep
 */
#include "tmc4671.h"

void stopMovement(TMC4671_Controller *tmc4671_controller){

	if(tmc4671_controller->target_position > tmc4671_controller->current_position + STOP_TOLERANCE_DECODER_COUNT){
		setIncrementalTargetPosition(tmc4671_controller, STOP_TOLERANCE_DECODER_COUNT);
	}

	if(tmc4671_controller->target_position < tmc4671_controller->current_position - STOP_TOLERANCE_DECODER_COUNT){
		setIncrementalTargetPosition(tmc4671_controller, -STOP_TOLERANCE_DECODER_COUNT);
	}
	tmc4671_controller->target_velocity = 0;
}

//---------------------------------------------------------------------------------------------------------
void setWrongCommandFlag(TMC4671_Controller *tmc4671_controller, bool is_wrong){
	tmc4671_controller->tmc_flags.wrong_command = is_wrong;
}

//---------------------------------------------------------------------------------------------------------
TMCStatusFlags getEventStatus(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_flags;
}

uint16_t getEventStatusWord_TMC(TMC4671_Controller *tmc4671_controller) {
    uint16_t status_word = 0;

    switch(tmc4671_controller->tmc_flags.limits) {
        case NO_LIMIT:
            break;
        case HARD_POSITIVE:
            status_word |= 0x0001 << 0;
            break;
        case HARD_NEGATIVE:
            status_word |= 0x0001 << 1;
            break;
        case SOFT_POSITIVE:
            status_word |= 0x0001 << 2;
            break;
        case SOFT_NEGATIVE:
            status_word |= 0x0001 << 3;
            break;
    }

    if (tmc4671_controller->tmc_flags.target_reached) {
        status_word |= 0x0001 << 4;
    }
    if (tmc4671_controller->tmc_flags.homing_done) {
        status_word |= 0x0001 << 5;
    }
    if (tmc4671_controller->tmc_flags.servo_enable) {
        status_word |= 0x0001 << 6;
    }
    if (tmc4671_controller->tmc_flags.servo_ready) {
        status_word |= 0x0001 << 7;
    }

    switch(tmc4671_controller->tmc_flags.faults) {
        case NO_FAULT:
            break;
        case SHORT_CIRCUIT:
            status_word |= 0x0001 << 8;
            break;
        case UNDER_VOLTAGE:
            status_word |= 0x0001 << 9;
            break;
        case OVER_TEMPERATURE:
            status_word |= 0x0001 << 10;
            break;
        case UNKNOWN:
            status_word |= 0x0001 << 11;
            break;
    }

    if (tmc4671_controller->tmc_flags.wrong_command) {
        status_word |= 0x0001 << 12;
    }

    return status_word;
}

//---------------------------------------------------------------------------------------------------------
void clearFaults(TMC4671_Controller *tmc4671_controller){
	tmc4671_controller->tmc_flags.limits = NO_LIMIT;
}

//---------------------------------------------------------------------------------------------------------

void setEncoderResolution(TMC4671_Controller *tmc4671_controller, uint32_t resolution_nanometers){
	tmc4671_controller->tmc_parameters.encoder_resolution = resolution_nanometers;

	uint32_t abn_decoder_ppr = round((1000*TOTAL_MAPPED_TRAVEL_MICRONS)/resolution_nanometers);
	write_register_tmc4671(TMC4671_ABN_DECODER_PPR, abn_decoder_ppr);
}

uint32_t getEncoderResolution(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.encoder_resolution;
}
//---------------------------------------------------------------------------------------------------------

void setEncoderDirection(TMC4671_Controller *tmc4671_controller, bool direction){
	tmc4671_controller->tmc_parameters.encoder_direction = (uint32_t)direction;

	uint32_t encoder_dir = (uint32_t)direction;
	encoder_dir &= 0x00000001;
	uint32_t abn_decoder_mode = SET_ABN_DECODER_MODE | (encoder_dir << 12);
	write_register_tmc4671(TMC4671_ABN_DECODER_MODE, abn_decoder_mode);
}

bool getEncoderDirection(TMC4671_Controller *tmc4671_controller){
	if(tmc4671_controller->tmc_parameters.encoder_direction == 1){
		return true;
	}else{
		return false;
	}
}
//---------------------------------------------------------------------------------------------------------

uint32_t getCoilCurrent(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->motor_current;
}
//---------------------------------------------------------------------------------------------------------

void setCurrentLimitHoming(TMC4671_Controller *tmc4671_controller, uint32_t current_limit){
	tmc4671_controller->tmc_parameters.current_limit_homing = current_limit;
}

uint32_t getCurrentLimitHoming(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.current_limit_homing;
}
//---------------------------------------------------------------------------------------------------------

void setCurrentLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t current_limit){
	tmc4671_controller->tmc_parameters.current_limit_servo = current_limit;
}

uint32_t getCurrentLimitServo(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.current_limit_servo;
}
//---------------------------------------------------------------------------------------------------------

void setVoltageLimitHoming(TMC4671_Controller *tmc4671_controller, uint32_t voltage_limit){
	tmc4671_controller->tmc_parameters.voltage_limit_homing = voltage_limit;
}

uint32_t getVoltageLimitHoming(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.voltage_limit_homing;
}
//---------------------------------------------------------------------------------------------------------

void setVelocityLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t velocity_limit){
	tmc4671_controller->tmc_parameters.velocity_limit_servo = velocity_limit;
	write_register_tmc4671(TMC4671_PID_VELOCITY_LIMIT, velocity_limit);
}

uint32_t getVelocityLimitServo(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.velocity_limit_servo;
}
//---------------------------------------------------------------------------------------------------------

void setAccelerationLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t acceleration_limit){
	tmc4671_controller->tmc_parameters.acceleration_limit_servo = acceleration_limit;
}

uint32_t getAccelerationLimitServo(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.acceleration_limit_servo;
}
//---------------------------------------------------------------------------------------------------------

void setTorqueLimitServo(TMC4671_Controller *tmc4671_controller, uint32_t torque_limit){
	tmc4671_controller->tmc_parameters.torque_limit_servo = torque_limit;
	write_register_tmc4671(TMC4671_PID_TORQUE_FLUX_LIMITS, torque_limit);
}

uint32_t getTorqueLimitServo(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.torque_limit_servo;
}
//---------------------------------------------------------------------------------------------------------

void setSoftPositiveLimit(TMC4671_Controller *tmc4671_controller, int32_t limit_microns){
	tmc4671_controller->tmc_parameters.soft_limit_positive = (int32_t)(limit_microns*(TOTAL_DECODER_REGISTER_COUNT/TOTAL_MAPPED_TRAVEL_MICRONS)) + tmc4671_controller->tmc_parameters.encoder_zero_offset;
	write_register_tmc4671(TMC4671_PID_POSITION_LIMIT_HIGH, tmc4671_controller->tmc_parameters.soft_limit_positive);
}

int32_t getSoftPositiveLimit(TMC4671_Controller *tmc4671_controller){
	int32_t pos_limit = (tmc4671_controller->tmc_parameters.soft_limit_positive - tmc4671_controller->tmc_parameters.encoder_zero_offset);
	pos_limit = (int32_t)((pos_limit*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return pos_limit;
}
//---------------------------------------------------------------------------------------------------------

void setSoftNegativeLimit(TMC4671_Controller *tmc4671_controller, int32_t limit_microns){
	tmc4671_controller->tmc_parameters.soft_limit_negative = (int32_t)(limit_microns*(TOTAL_DECODER_REGISTER_COUNT/TOTAL_MAPPED_TRAVEL_MICRONS)) + tmc4671_controller->tmc_parameters.encoder_zero_offset;
	write_register_tmc4671(TMC4671_PID_POSITION_LIMIT_LOW, tmc4671_controller->tmc_parameters.soft_limit_negative);
}

int32_t getSoftNegativeLimit(TMC4671_Controller *tmc4671_controller){
	int32_t neg_limit = (tmc4671_controller->tmc_parameters.soft_limit_negative - tmc4671_controller->tmc_parameters.encoder_zero_offset);
	neg_limit = (int32_t)((neg_limit*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return neg_limit;
}
//---------------------------------------------------------------------------------------------------------

void setZeroOffset(TMC4671_Controller *tmc4671_controller, int32_t offset_microns){
	tmc4671_controller->tmc_parameters.encoder_zero_offset = (int32_t)(offset_microns * (TOTAL_DECODER_REGISTER_COUNT/TOTAL_MAPPED_TRAVEL_MICRONS));
}

int32_t getZeroOffset(TMC4671_Controller *tmc4671_controller){
	int32_t offset_microns = tmc4671_controller->tmc_parameters.encoder_zero_offset;
	offset_microns = (int32_t)((offset_microns*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return offset_microns;
}
//---------------------------------------------------------------------------------------------------------

int32_t getActualPosition(TMC4671_Controller *tmc4671_controller){
	int32_t actual_microns = (tmc4671_controller->current_position - tmc4671_controller->tmc_parameters.encoder_zero_offset);
	actual_microns = (int32_t)((actual_microns*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return actual_microns;
}
//---------------------------------------------------------------------------------------------------------

void setAbsoluteTargetPosition(TMC4671_Controller *tmc4671_controller, int32_t target_microns){
	int32_t target_pos = (int32_t)(target_microns*(TOTAL_DECODER_REGISTER_COUNT/TOTAL_MAPPED_TRAVEL_MICRONS)) + tmc4671_controller->tmc_parameters.encoder_zero_offset;

	if(target_pos >= tmc4671_controller->tmc_parameters.soft_limit_positive){
		tmc4671_controller->tmc_flags.limits = SOFT_POSITIVE;
	}else if(target_pos <= tmc4671_controller->tmc_parameters.soft_limit_negative){
		tmc4671_controller->tmc_flags.limits = SOFT_NEGATIVE;
	}else{
		tmc4671_controller->target_position = target_pos;
		//add logic for motion profiler
		StartVelocityProfile(&tmc4671_controller->speed_profile, (double)tmc4671_controller->current_position, (double)tmc4671_controller->target_position);
	}
}

void setIncrementalTargetPosition(TMC4671_Controller *tmc4671_controller, int32_t target_microns){
	int32_t current_microns = getActualPosition(tmc4671_controller);
	setAbsoluteTargetPosition(tmc4671_controller, current_microns + target_microns);
}

int32_t getTargetPosition(TMC4671_Controller *tmc4671_controller){
	int32_t target_pos = (tmc4671_controller->target_position - tmc4671_controller->tmc_parameters.encoder_zero_offset);
	target_pos = (int32_t)((target_pos*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return target_pos;
}
//---------------------------------------------------------------------------------------------------------

int32_t getActualVelocity(TMC4671_Controller *tmc4671_controller){
	int32_t actual_microns_per_second = tmc4671_controller->current_velocity;
	actual_microns_per_second = (int32_t)((actual_microns_per_second*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return actual_microns_per_second;
}
//---------------------------------------------------------------------------------------------------------

void setTargetVelocity(TMC4671_Controller *tmc4671_controller, int32_t target_microns_per_second){
	int32_t target_pos_per_second = (int32_t)(target_microns_per_second*(TOTAL_DECODER_REGISTER_COUNT/TOTAL_MAPPED_TRAVEL_MICRONS));
	tmc4671_controller->target_velocity = target_pos_per_second;
}

int32_t getTargetVelocity(TMC4671_Controller *tmc4671_controller){
	int32_t target_microns_per_second = tmc4671_controller->target_velocity;
	target_microns_per_second = (int32_t)((target_microns_per_second*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return target_microns_per_second;
}
//---------------------------------------------------------------------------------------------------------

void setMaxPositionError(TMC4671_Controller *tmc4671_controller, uint32_t error_microns){
	tmc4671_controller->tmc_parameters.position_error_limit = (uint32_t)(error_microns * (TOTAL_DECODER_REGISTER_COUNT/TOTAL_MAPPED_TRAVEL_MICRONS));
}

uint32_t getMaxPositionError(TMC4671_Controller *tmc4671_controller){
	uint32_t error_microns = tmc4671_controller->tmc_parameters.position_error_limit;
	error_microns = (uint32_t)((error_microns*TOTAL_MAPPED_TRAVEL_MICRONS)/TOTAL_DECODER_REGISTER_COUNT);
	return error_microns;
}
//---------------------------------------------------------------------------------------------------------

void setCurrentGainP(TMC4671_Controller *tmc4671_controller, uint32_t gain){
	tmc4671_controller->tmc_parameters.current_P_gain = gain;
	uint32_t P_gain = tmc4671_controller->tmc_parameters.current_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.current_I_gain;
	I_gain &= 0x0000FFFF;

	uint32_t pid_torque_P_torque_I = (P_gain << 16) | I_gain;
	write_register_tmc4671(TMC4671_PID_TORQUE_P_TORQUE_I, pid_torque_P_torque_I);
	write_register_tmc4671(TMC4671_PID_FLUX_P_FLUX_I, pid_torque_P_torque_I);
}

uint32_t getCurrentGainP(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.current_P_gain;
}
//---------------------------------------------------------------------------------------------------------

void setCurrentGainI(TMC4671_Controller *tmc4671_controller, uint32_t gain){
	tmc4671_controller->tmc_parameters.current_I_gain = gain;
	uint32_t P_gain = tmc4671_controller->tmc_parameters.current_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.current_I_gain;
	I_gain &= 0x0000FFFF;

	uint32_t pid_torque_P_torque_I = (P_gain << 16) | I_gain;
	write_register_tmc4671(TMC4671_PID_TORQUE_P_TORQUE_I, pid_torque_P_torque_I);
	write_register_tmc4671(TMC4671_PID_FLUX_P_FLUX_I, pid_torque_P_torque_I);
}

uint32_t getCurrentGainI(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.current_I_gain;
}
//---------------------------------------------------------------------------------------------------------

void setVelocityGainP(TMC4671_Controller *tmc4671_controller, uint32_t gain){
	tmc4671_controller->tmc_parameters.velocity_P_gain = gain;
	uint32_t P_gain = tmc4671_controller->tmc_parameters.velocity_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.velocity_I_gain;
	I_gain &= 0x0000FFFF;

	uint32_t pid_velocity_P_velocity_I = (P_gain << 16) | I_gain;
	write_register_tmc4671(TMC4671_PID_VELOCITY_P_VELOCITY_I, pid_velocity_P_velocity_I);
}

uint32_t getVelocityGainP(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.velocity_P_gain;
}
//---------------------------------------------------------------------------------------------------------

void setVelocityGainI(TMC4671_Controller *tmc4671_controller, uint32_t gain){
	tmc4671_controller->tmc_parameters.velocity_I_gain = gain;
	uint32_t P_gain = tmc4671_controller->tmc_parameters.velocity_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.velocity_I_gain;
	I_gain &= 0x0000FFFF;

	uint32_t pid_velocity_P_velocity_I = (P_gain << 16) | I_gain;
	write_register_tmc4671(TMC4671_PID_VELOCITY_P_VELOCITY_I, pid_velocity_P_velocity_I);
}

uint32_t getVelocityGainI(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.velocity_I_gain;
}
//---------------------------------------------------------------------------------------------------------

void setPositionGainP(TMC4671_Controller *tmc4671_controller, uint32_t gain){
	tmc4671_controller->tmc_parameters.position_P_gain = gain;
	uint32_t P_gain = tmc4671_controller->tmc_parameters.position_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.position_I_gain;
	I_gain &= 0x0000FFFF;

	uint32_t pid_position_P_position_I = (P_gain << 16) | I_gain;
	write_register_tmc4671(TMC4671_PID_POSITION_P_POSITION_I, pid_position_P_position_I);
}

uint32_t getPositionGainP(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.position_P_gain;
}
//---------------------------------------------------------------------------------------------------------

void setPositionGainI(TMC4671_Controller *tmc4671_controller, uint32_t gain){
	tmc4671_controller->tmc_parameters.position_I_gain = gain;
	uint32_t P_gain = tmc4671_controller->tmc_parameters.position_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.position_I_gain;
	I_gain &= 0x0000FFFF;

	uint32_t pid_position_P_position_I = (P_gain << 16) | I_gain;
	write_register_tmc4671(TMC4671_PID_POSITION_P_POSITION_I, pid_position_P_position_I);
}

uint32_t getPositionGainI(TMC4671_Controller *tmc4671_controller){
	return tmc4671_controller->tmc_parameters.position_I_gain;
}
//---------------------------------------------------------------------------------------------------------

void servoEnable(TMC4671_Controller *tmc4671_controller, bool enable){
	tmc4671_controller->target_position = tmc4671_controller->current_position;
	tmc4671_controller->tmc_flags.servo_enable = enable;
}
//---------------------------------------------------------------------------------------------------------

bool initializeTMC4671(TMC4671_Controller *tmc4671_controller){
	uint8_t temp_bytes[4];

	EEPROM_Read(DEF_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);
	uint32_t default_available = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);
	uint32_t saved_available = bytes2uInt(temp_bytes);

	if(default_available == 0xFFFFFFFF){
		saveDefaultParameters();
	}

	if(saved_available == 0xFFFFFFFF){
		makeDefaultParametersCurrent();
	}

	tmc4671_controller->current_position = 0;
	tmc4671_controller->current_velocity = 0;
	tmc4671_controller->motor_current = 0;
	tmc4671_controller->target_position = 0;
	tmc4671_controller->target_velocity = 0;

	tmc4671_controller->tmc_flags.servo_ready = false;
	tmc4671_controller->tmc_flags.servo_enable = false;
	tmc4671_controller->tmc_flags.homing_done = false;
	tmc4671_controller->tmc_flags.target_reached = false;
	tmc4671_controller->tmc_flags.limits = NO_LIMIT;
	tmc4671_controller->tmc_flags.faults = NO_FAULT;
	tmc4671_controller->tmc_flags.wrong_command = false;

	loadParameters(tmc4671_controller);

	if(!initializeTMC6100()){
		return false;
	}
	HAL_Delay(200);

	if(!is_tmc4671_connected()){
		return false;
	}
	HAL_Delay(200);

	if(!setup_controller(tmc4671_controller)){
		return false;
	}

	HAL_Delay(200);

	if(!setup_PID(tmc4671_controller)){
		return false;
	}

	InitializeVelocityProfile(&tmc4671_controller->speed_profile, DEFAULT_VELOCITY_LIMIT_SERVO, tmc4671_controller->tmc_parameters.acceleration_limit_servo, DEFAULT_VELOCITY_STARTUP_SERVO);

	return true;
}
//---------------------------------------------------------------------------------------------------------

void servoRun(TMC4671_Controller *tmc4671_controller){
	tmc4671_controller->current_position = get_position();
	tmc4671_controller->current_velocity = get_velocity();
	tmc4671_controller->motor_current = get_motor_current();

	if(tmc4671_controller->motor_current > tmc4671_controller->tmc_parameters.current_centre_value + tmc4671_controller->tmc_parameters.current_limit_servo){
		tmc4671_controller->tmc_flags.limits = HARD_POSITIVE;
	}

	if(tmc4671_controller->motor_current < tmc4671_controller->tmc_parameters.current_centre_value - tmc4671_controller->tmc_parameters.current_limit_servo){
		tmc4671_controller->tmc_flags.limits = HARD_NEGATIVE;
	}

	if((tmc4671_controller->tmc_flags.limits == HARD_POSITIVE) || tmc4671_controller->tmc_flags.limits == HARD_NEGATIVE){
		control_disable();
		setTargetVelocity(tmc4671_controller, 0);
		setAbsoluteTargetPosition(tmc4671_controller, 0);
	}else{
		if(tmc4671_controller->tmc_flags.servo_enable){
			control_enable();
		}else{
			control_disable();
		}

		int32_t target_vel = (int32_t)ComputeProfileSpeed(&tmc4671_controller->speed_profile, (double)tmc4671_controller->current_position);
		if(target_vel != 0){
			set_velocity(target_vel);
		}else{
			set_position(tmc4671_controller->target_position);
			SetVelocityProfileZero(&tmc4671_controller->speed_profile);
		}
	}

	GPIO_PinState check = HAL_GPIO_ReadPin(TMC_STATUS_GPIO_Port, TMC_STATUS_Pin);
	if(check == GPIO_PIN_SET){
		tmc4671_controller->tmc_flags.faults = checkFaults();
	}else{
		tmc4671_controller->tmc_flags.faults = NO_FAULT;
	}

	if(!tmc4671_controller->tmc_flags.servo_enable || (tmc4671_controller->tmc_flags.faults != NO_FAULT) || !tmc4671_controller->tmc_flags.homing_done || (tmc4671_controller->tmc_flags.limits != NO_LIMIT)){
		tmc4671_controller->tmc_flags.servo_ready = false;
	}else{
		tmc4671_controller->tmc_flags.servo_ready = true;
	}

	int pos_error = (int)get_position_error();
	if(abs(pos_error) <= tmc4671_controller->tmc_parameters.position_error_limit){
		tmc4671_controller->tmc_flags.target_reached = true;
	}else{
		tmc4671_controller->tmc_flags.target_reached = false;
	}
}
//---------------------------------------------------------------------------------------------------------

void startHoming(TMC4671_Controller *tmc4671_controller){
	// ABN encoder settings
	uint32_t encoder_dir = tmc4671_controller->tmc_parameters.encoder_direction;
	encoder_dir &= 0x00000001;
	uint32_t abn_decoder_mode = SET_ABN_DECODER_MODE | (encoder_dir << 12);
	write_register_tmc4671(TMC4671_ABN_DECODER_MODE, abn_decoder_mode);
	//---------------------

	tmc4671_controller->tmc_flags.homing_done = false;
	control_enable();
	write_register_tmc4671(TMC4671_MODE_RAMP_MODE_MOTION, SET_MOTION_MODE_UQ_UD);
	write_register_tmc4671(TMC4671_PHI_E_SELECTION, SET_PHI_E_SELECTION_OPENLOOP);

	//int32_t homing_uq_ud_ext_limit = (int32_t)tmc4671_controller->tmc_parameters.voltage_limit_homing;

	index_found = false;
	while(!index_found){
		if(!index_found){
			int32_t uq_ud_val = 0;
			while(tmc4671_controller->motor_current < tmc4671_controller->tmc_parameters.current_centre_value + tmc4671_controller->tmc_parameters.current_limit_homing){
				//if(uq_ud_val < homing_uq_ud_ext_limit){
					uq_ud_val += 1;
				//}
				write_register_tmc4671(TMC4671_UQ_UD_EXT, (uq_ud_val << 16));
				if(index_found){
					break;
				}
				tmc4671_controller->motor_current = get_motor_current();
			}
			write_register_tmc4671(TMC4671_UQ_UD_EXT, 0);
			HAL_Delay(50);
		}

		if(!index_found){
			int32_t uq_ud_val = 0;
			while(tmc4671_controller->motor_current > tmc4671_controller->tmc_parameters.current_centre_value - tmc4671_controller->tmc_parameters.current_limit_homing){
				//if(uq_ud_val > -homing_uq_ud_ext_limit){
					uq_ud_val -= 1;
				//}
				write_register_tmc4671(TMC4671_UQ_UD_EXT, (uq_ud_val << 16));
				if(index_found){
					break;
				}
				tmc4671_controller->motor_current = get_motor_current();
			}
			write_register_tmc4671(TMC4671_UQ_UD_EXT, 0);
			HAL_Delay(50);
		}
	}
	tmc4671_controller->tmc_flags.homing_done = true;
	tmc4671_controller->target_position = 0;
	tmc4671_controller->target_velocity = 0;

	// ABN encoder settings
	encoder_dir = tmc4671_controller->tmc_parameters.encoder_direction;
	encoder_dir &= 0x00000001;
	abn_decoder_mode = SET_ABN_DECODER_MODE_DIS | (encoder_dir << 12);
	write_register_tmc4671(TMC4671_ABN_DECODER_MODE, abn_decoder_mode);
}
//---------------------------------------------------------------------------------------------------------

bool is_tmc4671_connected(){
	uint8_t xfrbuf[5];
	HAL_StatusTypeDef ret;

	xfrbuf[0]= WRITE_MASK | TMC4671_CHIPINFO_ADDR;
	xfrbuf[1]= 0x00;
	xfrbuf[2]= 0x00;
	xfrbuf[3]= 0x00;
	xfrbuf[4]= 0x00;

	TMC4671_CS_SET();
	ret = HAL_SPI_TransmitReceive(&TMC_PORT, xfrbuf, xfrbuf, 5, HAL_MAX_DELAY);
	TMC4671_CS_RESET();

	if(ret != HAL_OK){
		return false;
	}else{
		HAL_Delay(20);
		uint32_t ret = read_register_tmc4671(TMC4671_CHIPINFO_DATA);
		if(ret == 0x34363731){
			return true;
		}else{
			return false;
		}
	}
}
//---------------------------------------------------------------------------------------------------------

uint32_t read_register_tmc4671(uint8_t address){
	uint8_t xfrbuf[5];
	HAL_StatusTypeDef ret;

	xfrbuf[0] = READ_MASK | address;
	xfrbuf[1] = 0x00;
	xfrbuf[2] = 0x00;
	xfrbuf[3] = 0x00;
	xfrbuf[4] = 0x00;

	TMC4671_CS_SET();
	ret = HAL_SPI_TransmitReceive(&TMC_PORT, xfrbuf, xfrbuf, 5, HAL_MAX_DELAY);
	TMC4671_CS_RESET();

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
//---------------------------------------------------------------------------------------------------------

bool write_register_tmc4671(uint8_t address, uint32_t data){
	uint8_t xfrbuf[5];
	HAL_StatusTypeDef ret;

	xfrbuf[0] = WRITE_MASK | address;
	xfrbuf[1] = (uint8_t)((data >> 24) & 0xFF);
	xfrbuf[2] = (uint8_t)((data >> 16) & 0xFF);
	xfrbuf[3] = (uint8_t)((data >> 8) & 0xFF);
	xfrbuf[4] = (uint8_t)((data) & 0xFF);

	TMC4671_CS_SET();
	ret = HAL_SPI_TransmitReceive(&TMC_PORT, xfrbuf, xfrbuf, 5, HAL_MAX_DELAY);
	TMC4671_CS_RESET();

	if(ret != HAL_OK){
		return false;
	}else{
		return true;
	}
}
//---------------------------------------------------------------------------------------------------------

bool setup_controller(TMC4671_Controller *tmc4671_controller){

	// Motor type &  PWM configuration
	if(!write_register_tmc4671(TMC4671_MOTOR_TYPE_N_POLE_PAIRS, SET_MOTOR_TYPE_N_POLE_PAIRS)){return false;}
	if(!write_register_tmc4671(TMC4671_PWM_POLARITIES, SET_PWM_POLARITIES)){return false;}
	if(!write_register_tmc4671(TMC4671_PWM_MAXCNT, SET_PWM_MAXCNT)){return false;}
	if(!write_register_tmc4671(TMC4671_PWM_BBM_H_BBM_L,  SET_PWM_BBM_H_BBM_L)){return false;}
	if(!write_register_tmc4671(TMC4671_PWM_SV_CHOP, SET_PWM_SV_CHOP)){return false;}

	// ADC configuration
	if(!write_register_tmc4671(TMC4671_ADC_I_SELECT, SET_ADC_I_SELECT)){return false;}
	if(!write_register_tmc4671(TMC4671_dsADC_MCFG_B_MCFG_A, SET_dsADC_MCFG_B_MCFG_A)){return false;}
	if(!write_register_tmc4671(TMC4671_dsADC_MCLK_A, SET_dsADC_MCLK_A)){return false;}
	if(!write_register_tmc4671(TMC4671_dsADC_MCLK_B, SET_dsADC_MCLK_B)){return false;}
	if(!write_register_tmc4671(TMC4671_dsADC_MDEC_B_MDEC_A, SET_dsADC_MDEC_B_MDEC_A)){return false;}
	if(!write_register_tmc4671(TMC4671_ADC_I0_SCALE_OFFSET, SET_ADC_I0_SCALE_OFFSET)){return false;}
	if(!write_register_tmc4671(TMC4671_ADC_I1_SCALE_OFFSET, SET_ADC_I1_SCALE_OFFSET)){return false;}

	// ABN encoder settings
	uint32_t encoder_dir = tmc4671_controller->tmc_parameters.encoder_direction;
	encoder_dir &= 0x00000001;
	uint32_t abn_decoder_mode = SET_ABN_DECODER_MODE | (encoder_dir << 12);

	if(!write_register_tmc4671(TMC4671_ABN_DECODER_MODE, abn_decoder_mode)){return false;}

	uint32_t abn_decoder_ppr = round((1000*TOTAL_MAPPED_TRAVEL_MICRONS)/tmc4671_controller->tmc_parameters.encoder_resolution);

	if(!write_register_tmc4671(TMC4671_ABN_DECODER_PPR, abn_decoder_ppr)){return false;}
	if(!write_register_tmc4671(TMC4671_ABN_DECODER_COUNT, SET_ABN_DECODER_COUNT)){return false;}
	if(!write_register_tmc4671(TMC4671_ABN_DECODER_COUNT_N, SET_ABN_DECODER_COUNT_N)){return false;}
	if(!write_register_tmc4671(TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, SET_ABN_DECODER_PHI_E_PHI_M_OFFSET)){return false;}

	return true;
}
//---------------------------------------------------------------------------------------------------------

bool setup_PID(TMC4671_Controller *tmc4671_controller){
	//selectors
	if(!write_register_tmc4671(TMC4671_PHI_E_SELECTION, SET_PHI_E_SELECTION_ABN)){return false;}
	if(!write_register_tmc4671(TMC4671_VELOCITY_SELECTION, SET_VELOCITY_SELECTION)){return false;}
	if(!write_register_tmc4671(TMC4671_POSITION_SELECTION, SET_POSITION_SELECTION)){return false;}
	if(!write_register_tmc4671(TMC4671_MODE_RAMP_MODE_MOTION, SET_MOTION_MODE_POSITION)){return false;}
	if(!write_register_tmc4671(TMC4671_ADC_I_SELECT, SET_ADC_I_SELECT)){return false;}

	//limits
	uint32_t pid_torque_flux_limits = tmc4671_controller->tmc_parameters.torque_limit_servo;
	uint32_t pid_velocity_limit = tmc4671_controller->tmc_parameters.velocity_limit_servo;
	int32_t pid_position_limit_low = tmc4671_controller->tmc_parameters.soft_limit_negative;
	int32_t pid_position_limit_high = tmc4671_controller->tmc_parameters.soft_limit_positive;

	if(!write_register_tmc4671(TMC4671_PIDOUT_UQ_UD_LIMITS, SET_PIDOUT_UQ_UD_LIMITS)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_TORQUE_FLUX_LIMITS, pid_torque_flux_limits)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_VELOCITY_LIMIT, pid_velocity_limit)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_POSITION_LIMIT_LOW, pid_position_limit_low)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_POSITION_LIMIT_HIGH, pid_position_limit_high)){return false;}

	//PID params
	uint32_t P_gain = tmc4671_controller->tmc_parameters.current_P_gain;
	P_gain &= 0x0000FFFF;
	uint32_t I_gain = tmc4671_controller->tmc_parameters.current_I_gain;
	I_gain &= 0x0000FFFF;
	uint32_t pid_torque_P_torque_I = (P_gain << 16) | I_gain;

	P_gain = tmc4671_controller->tmc_parameters.velocity_P_gain;
	P_gain &= 0x0000FFFF;
	I_gain = tmc4671_controller->tmc_parameters.velocity_I_gain;
	I_gain &= 0x0000FFFF;
	uint32_t pid_velocity_P_velocity_I = (P_gain << 16) | I_gain;

	P_gain = tmc4671_controller->tmc_parameters.position_P_gain;
	P_gain &= 0x0000FFFF;
	I_gain = tmc4671_controller->tmc_parameters.position_I_gain;
	I_gain &= 0x0000FFFF;
	uint32_t pid_position_P_position_I = (P_gain << 16) | I_gain;

	if(!write_register_tmc4671(TMC4671_PID_TORQUE_P_TORQUE_I, pid_torque_P_torque_I)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_FLUX_P_FLUX_I, pid_torque_P_torque_I)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_VELOCITY_P_VELOCITY_I, pid_velocity_P_velocity_I)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_POSITION_P_POSITION_I, pid_position_P_position_I)){return false;}

	//initialization
	if(!write_register_tmc4671(TMC4671_PID_POSITION_ACTUAL, SET_PID_POSITION_ACTUAL_ZERO)){return false;}

	return true;
}
//---------------------------------------------------------------------------------------------------------

bool set_velocity(int32_t velocity){
	if(!write_register_tmc4671(TMC4671_MODE_RAMP_MODE_MOTION, SET_MOTION_MODE_VELOCITY)){return false;}
	if(!write_register_tmc4671(TMC4671_PHI_E_SELECTION, SET_PHI_E_SELECTION_ABN)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_VELOCITY_TARGET, velocity)){return false;}
	return true;
}
//---------------------------------------------------------------------------------------------------------

bool set_position(int32_t position){
	if(!write_register_tmc4671(TMC4671_MODE_RAMP_MODE_MOTION, SET_MOTION_MODE_POSITION)){return false;}
	if(!write_register_tmc4671(TMC4671_PHI_E_SELECTION, SET_PHI_E_SELECTION_ABN)){return false;}
	if(!write_register_tmc4671(TMC4671_PID_POSITION_TARGET, position)){return false;}
	return true;
}
//---------------------------------------------------------------------------------------------------------

int32_t get_velocity(){
	uint32_t uVel = read_register_tmc4671(TMC4671_PID_VELOCITY_ACTUAL);
	if(uVel != 0xFFFFFFFF){
		return (int32_t)uVel;
	}else{
		return 0;
	}
}
//---------------------------------------------------------------------------------------------------------

int32_t get_position(){
	uint32_t uPos = read_register_tmc4671(TMC4671_PID_POSITION_ACTUAL);
	if(uPos != 0xFFFFFFFF){
		return (int32_t)uPos;
	}else{
		return 0;
	}
}
//---------------------------------------------------------------------------------------------------------

int32_t get_position_error(){
	if(!write_register_tmc4671(TMC4671_PID_ERROR_ADDR, PID_POSITION_ERROR)){return 0;}
	int32_t pos_error = read_register_tmc4671(TMC4671_PID_ERROR_DATA);

	return pos_error;
}
//---------------------------------------------------------------------------------------------------------

uint32_t get_motor_current(){
	if(!write_register_tmc4671(TMC4671_ADC_RAW_ADDR, 0x00000000)){return 0;}
	uint32_t raw_current = read_register_tmc4671(TMC4671_ADC_RAW_DATA) & 0xFFFF;

	return raw_current;
}
//---------------------------------------------------------------------------------------------------------

void control_enable(){
	HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_SET);
}

void control_disable(){
	HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_RESET);
}
//---------------------------------------------------------------------------------------------------------

uint32_t getFirmwareVersion(){
	uint8_t temp_bytes[4];

	EEPROM_Read(DEF_ADDR_VERSION_NUMBER, temp_bytes, 4);

	uint32_t version_no = bytes2uInt(temp_bytes);
	return version_no;
}

//---------------------------------------------------------------------------------------------------------

void loadDefaultParameters(TMC4671_Controller *tmc4671_controller){
	uint8_t temp_bytes[4];

	EEPROM_Read(DEF_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.encoder_resolution = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_ENCODER_DIRECTION, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.encoder_direction = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_ENCODER_ZERO_OFFSET, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.encoder_zero_offset = bytes2sInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_SOFT_LIMIT_POSITIVE, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.soft_limit_positive = bytes2sInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_SOFT_LIMIT_NEGATIVE, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.soft_limit_negative = bytes2sInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_POSITION_ERROR_LIMIT, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.position_error_limit = bytes2uInt(temp_bytes);
	//-------------------------------------------------------------------

	EEPROM_Read(DEF_ADDR_CURRENT_P_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_P_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_CURRENT_I_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_I_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_VELOCITY_P_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.velocity_P_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_VELOCITY_I_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.velocity_I_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_POSITION_P_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.position_P_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_POSITION_I_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.position_I_gain = bytes2uInt(temp_bytes);
	//------------------------------------------------------------------

	EEPROM_Read(DEF_ADDR_CURRENT_CENTER_VAL, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_centre_value = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_CURRENT_LIMIT_HOMING, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_limit_homing = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_CURRENT_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_limit_servo = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_VOLTAGE_LIMIT_HOMING, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.voltage_limit_homing = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_VELOCITY_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.velocity_limit_servo = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_TORQUE_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.torque_limit_servo = bytes2uInt(temp_bytes);

	EEPROM_Read(DEF_ADDR_ACCEL_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.acceleration_limit_servo = bytes2uInt(temp_bytes);
}
//---------------------------------------------------------------------------------------------------------

void loadParameters(TMC4671_Controller *tmc4671_controller){
	uint8_t temp_bytes[4];

	EEPROM_Read(SAVE_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.encoder_resolution = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_ENCODER_DIRECTION, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.encoder_direction = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_ENCODER_ZERO_OFFSET, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.encoder_zero_offset = bytes2sInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_SOFT_LIMIT_POSITIVE, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.soft_limit_positive = bytes2sInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_SOFT_LIMIT_NEGATIVE, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.soft_limit_negative = bytes2sInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_POSITION_ERROR_LIMIT, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.position_error_limit = bytes2uInt(temp_bytes);
	//-------------------------------------------------------------------

	EEPROM_Read(SAVE_ADDR_CURRENT_P_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_P_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_CURRENT_I_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_I_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_VELOCITY_P_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.velocity_P_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_VELOCITY_I_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.velocity_I_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_POSITION_P_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.position_P_gain = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_POSITION_I_GAIN, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.position_I_gain = bytes2uInt(temp_bytes);
	//------------------------------------------------------------------

	EEPROM_Read(SAVE_ADDR_CURRENT_CENTER_VAL, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_centre_value = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_CURRENT_LIMIT_HOMING, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_limit_homing = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_CURRENT_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.current_limit_servo = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_VOLTAGE_LIMIT_HOMING, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.voltage_limit_homing = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_VELOCITY_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.velocity_limit_servo = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_TORQUE_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.torque_limit_servo = bytes2uInt(temp_bytes);

	EEPROM_Read(SAVE_ADDR_ACCEL_LIMIT_SERVO, temp_bytes, 4);
	tmc4671_controller->tmc_parameters.acceleration_limit_servo = bytes2uInt(temp_bytes);
}
//---------------------------------------------------------------------------------------------------------

void saveParameters(TMC4671_Controller *tmc4671_controller){
	Parameters tempParameters;

	tempParameters.encoder_resolution = tmc4671_controller->tmc_parameters.encoder_resolution;
	tempParameters.encoder_direction = tmc4671_controller->tmc_parameters.encoder_direction;
	tempParameters.encoder_zero_offset = tmc4671_controller->tmc_parameters.encoder_zero_offset;
	tempParameters.soft_limit_positive = tmc4671_controller->tmc_parameters.soft_limit_positive;
	tempParameters.soft_limit_negative = tmc4671_controller->tmc_parameters.soft_limit_negative;
	tempParameters.position_error_limit = tmc4671_controller->tmc_parameters.position_error_limit;

	tempParameters.current_P_gain = tmc4671_controller->tmc_parameters.current_P_gain;
	tempParameters.current_I_gain = tmc4671_controller->tmc_parameters.current_I_gain;
	tempParameters.velocity_P_gain = tmc4671_controller->tmc_parameters.velocity_P_gain;
	tempParameters.velocity_I_gain = tmc4671_controller->tmc_parameters.velocity_I_gain;
	tempParameters.position_P_gain = tmc4671_controller->tmc_parameters.position_P_gain;
	tempParameters.position_I_gain = tmc4671_controller->tmc_parameters.position_I_gain;

	tempParameters.current_centre_value = tmc4671_controller->tmc_parameters.current_centre_value;
	tempParameters.current_limit_homing = tmc4671_controller->tmc_parameters.current_limit_homing;
	tempParameters.current_limit_servo = tmc4671_controller->tmc_parameters.current_limit_servo;
	tempParameters.voltage_limit_homing = tmc4671_controller->tmc_parameters.voltage_limit_homing;
	tempParameters.velocity_limit_servo = tmc4671_controller->tmc_parameters.velocity_limit_servo;
	tempParameters.torque_limit_servo = tmc4671_controller->tmc_parameters.torque_limit_servo;
	tempParameters.acceleration_limit_servo = tmc4671_controller->tmc_parameters.acceleration_limit_servo;

	uint8_t temp_bytes[4];

	uInt2Bytes(temp_bytes, tempParameters.encoder_resolution);
	EEPROM_Write(SAVE_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.encoder_direction);
	EEPROM_Write(SAVE_ADDR_ENCODER_DIRECTION, temp_bytes, 4);

	sInt2Bytes(temp_bytes, tempParameters.encoder_zero_offset);
	EEPROM_Write(SAVE_ADDR_ENCODER_ZERO_OFFSET, temp_bytes, 4);

	sInt2Bytes(temp_bytes, tempParameters.soft_limit_positive);
	EEPROM_Write(SAVE_ADDR_SOFT_LIMIT_POSITIVE, temp_bytes, 4);

	sInt2Bytes(temp_bytes, tempParameters.soft_limit_negative);
	EEPROM_Write(SAVE_ADDR_SOFT_LIMIT_NEGATIVE, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.position_error_limit);
	EEPROM_Write(SAVE_ADDR_POSITION_ERROR_LIMIT, temp_bytes, 4);
	//-------------------------------------------------------------------
	uInt2Bytes(temp_bytes, tempParameters.current_P_gain);
	EEPROM_Write(SAVE_ADDR_CURRENT_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.current_I_gain);
	EEPROM_Write(SAVE_ADDR_CURRENT_I_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.velocity_P_gain);
	EEPROM_Write(SAVE_ADDR_VELOCITY_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.velocity_I_gain);
	EEPROM_Write(SAVE_ADDR_VELOCITY_I_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.position_P_gain);
	EEPROM_Write(SAVE_ADDR_POSITION_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.position_I_gain);
	EEPROM_Write(SAVE_ADDR_POSITION_I_GAIN, temp_bytes, 4);
	//------------------------------------------------------------------
	uInt2Bytes(temp_bytes, tempParameters.current_centre_value);
	EEPROM_Write(SAVE_ADDR_CURRENT_CENTER_VAL, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.current_limit_homing);
	EEPROM_Write(SAVE_ADDR_CURRENT_LIMIT_HOMING, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.current_limit_servo);
	EEPROM_Write(SAVE_ADDR_CURRENT_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.voltage_limit_homing);
	EEPROM_Write(SAVE_ADDR_VOLTAGE_LIMIT_HOMING, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.velocity_limit_servo);
	EEPROM_Write(SAVE_ADDR_VELOCITY_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.torque_limit_servo);
	EEPROM_Write(SAVE_ADDR_TORQUE_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, tempParameters.acceleration_limit_servo);
	EEPROM_Write(SAVE_ADDR_ACCEL_LIMIT_SERVO, temp_bytes, 4);
}
//---------------------------------------------------------------------------------------------------------

void makeDefaultParametersCurrent(){
	Parameters factoryDefaults;

	factoryDefaults.encoder_resolution = DEFAULT_ENCODER_RESOLUTION;
	factoryDefaults.encoder_direction = DEFAULT_ENCODER_DIRECTION;
	factoryDefaults.encoder_zero_offset = DEFAULT_ENCODER_ZERO_OFFSET;
	factoryDefaults.soft_limit_positive = DEFAULT_SOFT_LIMIT_POSITIVE;
	factoryDefaults.soft_limit_negative = DEFAULT_SOFT_LIMIT_NEGATIVE;
	factoryDefaults.position_error_limit = DEFAULT_POSITION_ERROR_LIMIT;

	factoryDefaults.current_P_gain = DEFAULT_CURRENT_P_GAIN;
	factoryDefaults.current_I_gain = DEFAULT_CURRENT_I_GAIN;
	factoryDefaults.velocity_P_gain = DEFAULT_VELOCITY_P_GAIN;
	factoryDefaults.velocity_I_gain = DEFAULT_VELOCITY_I_GAIN;
	factoryDefaults.position_P_gain = DEFAULT_POSITION_P_GAIN;
	factoryDefaults.position_I_gain = DEFAULT_POSITION_I_GAIN;

	factoryDefaults.current_centre_value = DEFAULT_CURRENT_CENTER_VAL;
	factoryDefaults.current_limit_homing = DEFAULT_CURRENT_LIMIT_HOMING;
	factoryDefaults.current_limit_servo = DEFAULT_CURRENT_LIMIT_SERVO;
	factoryDefaults.voltage_limit_homing = DEFAULT_VOLTAGE_LIMIT_HOMING;
	factoryDefaults.velocity_limit_servo = DEFAULT_VELOCITY_LIMIT_SERVO;
	factoryDefaults.torque_limit_servo = DEFAULT_TORQUE_LIMIT_SERVO;
	factoryDefaults.acceleration_limit_servo = DEFAULT_ACCEL_LIMIT_SERVO;

	uint8_t temp_bytes[4];

	uInt2Bytes(temp_bytes, factoryDefaults.encoder_resolution);
	EEPROM_Write(SAVE_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.encoder_direction);
	EEPROM_Write(SAVE_ADDR_ENCODER_DIRECTION, temp_bytes, 4);

	sInt2Bytes(temp_bytes, factoryDefaults.encoder_zero_offset);
	EEPROM_Write(SAVE_ADDR_ENCODER_ZERO_OFFSET, temp_bytes, 4);

	sInt2Bytes(temp_bytes, factoryDefaults.soft_limit_positive);
	EEPROM_Write(SAVE_ADDR_SOFT_LIMIT_POSITIVE, temp_bytes, 4);

	sInt2Bytes(temp_bytes, factoryDefaults.soft_limit_negative);
	EEPROM_Write(SAVE_ADDR_SOFT_LIMIT_NEGATIVE, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.position_error_limit);
	EEPROM_Write(SAVE_ADDR_POSITION_ERROR_LIMIT, temp_bytes, 4);
	//-------------------------------------------------------------------
	uInt2Bytes(temp_bytes, factoryDefaults.current_P_gain);
	EEPROM_Write(SAVE_ADDR_CURRENT_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.current_I_gain);
	EEPROM_Write(SAVE_ADDR_CURRENT_I_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.velocity_P_gain);
	EEPROM_Write(SAVE_ADDR_VELOCITY_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.velocity_I_gain);
	EEPROM_Write(SAVE_ADDR_VELOCITY_I_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.position_P_gain);
	EEPROM_Write(SAVE_ADDR_POSITION_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.position_I_gain);
	EEPROM_Write(SAVE_ADDR_POSITION_I_GAIN, temp_bytes, 4);
	//------------------------------------------------------------------
	uInt2Bytes(temp_bytes, factoryDefaults.current_centre_value);
	EEPROM_Write(SAVE_ADDR_CURRENT_CENTER_VAL, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.current_limit_homing);
	EEPROM_Write(SAVE_ADDR_CURRENT_LIMIT_HOMING, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.current_limit_servo);
	EEPROM_Write(SAVE_ADDR_CURRENT_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.voltage_limit_homing);
	EEPROM_Write(SAVE_ADDR_VOLTAGE_LIMIT_HOMING, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.velocity_limit_servo);
	EEPROM_Write(SAVE_ADDR_VELOCITY_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.torque_limit_servo);
	EEPROM_Write(SAVE_ADDR_TORQUE_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.acceleration_limit_servo);
	EEPROM_Write(SAVE_ADDR_ACCEL_LIMIT_SERVO, temp_bytes, 4);
}
//---------------------------------------------------------------------------------------------------------

void saveDefaultParameters(){
	Parameters factoryDefaults;

	factoryDefaults.encoder_resolution = DEFAULT_ENCODER_RESOLUTION;
	factoryDefaults.encoder_direction = DEFAULT_ENCODER_DIRECTION;
	factoryDefaults.encoder_zero_offset = DEFAULT_ENCODER_ZERO_OFFSET;
	factoryDefaults.soft_limit_positive = DEFAULT_SOFT_LIMIT_POSITIVE;
	factoryDefaults.soft_limit_negative = DEFAULT_SOFT_LIMIT_NEGATIVE;
	factoryDefaults.position_error_limit = DEFAULT_POSITION_ERROR_LIMIT;

	factoryDefaults.current_P_gain = DEFAULT_CURRENT_P_GAIN;
	factoryDefaults.current_I_gain = DEFAULT_CURRENT_I_GAIN;
	factoryDefaults.velocity_P_gain = DEFAULT_VELOCITY_P_GAIN;
	factoryDefaults.velocity_I_gain = DEFAULT_VELOCITY_I_GAIN;
	factoryDefaults.position_P_gain = DEFAULT_POSITION_P_GAIN;
	factoryDefaults.position_I_gain = DEFAULT_POSITION_I_GAIN;

	factoryDefaults.current_centre_value = DEFAULT_CURRENT_CENTER_VAL;
	factoryDefaults.current_limit_homing = DEFAULT_CURRENT_LIMIT_HOMING;
	factoryDefaults.current_limit_servo = DEFAULT_CURRENT_LIMIT_SERVO;
	factoryDefaults.voltage_limit_homing = DEFAULT_VOLTAGE_LIMIT_HOMING;
	factoryDefaults.velocity_limit_servo = DEFAULT_VELOCITY_LIMIT_SERVO;
	factoryDefaults.torque_limit_servo = DEFAULT_TORQUE_LIMIT_SERVO;
	factoryDefaults.acceleration_limit_servo = DEFAULT_ACCEL_LIMIT_SERVO;

	uint8_t temp_bytes[4];

	uInt2Bytes(temp_bytes, factoryDefaults.encoder_resolution);
	EEPROM_Write(DEF_ADDR_ENCODER_RESOLUTION, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.encoder_direction);
	EEPROM_Write(DEF_ADDR_ENCODER_DIRECTION, temp_bytes, 4);

	sInt2Bytes(temp_bytes, factoryDefaults.encoder_zero_offset);
	EEPROM_Write(DEF_ADDR_ENCODER_ZERO_OFFSET, temp_bytes, 4);

	sInt2Bytes(temp_bytes, factoryDefaults.soft_limit_positive);
	EEPROM_Write(DEF_ADDR_SOFT_LIMIT_POSITIVE, temp_bytes, 4);

	sInt2Bytes(temp_bytes, factoryDefaults.soft_limit_negative);
	EEPROM_Write(DEF_ADDR_SOFT_LIMIT_NEGATIVE, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.position_error_limit);
	EEPROM_Write(DEF_ADDR_POSITION_ERROR_LIMIT, temp_bytes, 4);
	//-------------------------------------------------------------------
	uInt2Bytes(temp_bytes, factoryDefaults.current_P_gain);
	EEPROM_Write(DEF_ADDR_CURRENT_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.current_I_gain);
	EEPROM_Write(DEF_ADDR_CURRENT_I_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.velocity_P_gain);
	EEPROM_Write(DEF_ADDR_VELOCITY_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.velocity_I_gain);
	EEPROM_Write(DEF_ADDR_VELOCITY_I_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.position_P_gain);
	EEPROM_Write(DEF_ADDR_POSITION_P_GAIN, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.position_I_gain);
	EEPROM_Write(DEF_ADDR_POSITION_I_GAIN, temp_bytes, 4);
	//------------------------------------------------------------------
	uInt2Bytes(temp_bytes, factoryDefaults.current_centre_value);
	EEPROM_Write(DEF_ADDR_CURRENT_CENTER_VAL, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.current_limit_homing);
	EEPROM_Write(DEF_ADDR_CURRENT_LIMIT_HOMING, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.current_limit_servo);
	EEPROM_Write(DEF_ADDR_CURRENT_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.voltage_limit_homing);
	EEPROM_Write(DEF_ADDR_VOLTAGE_LIMIT_HOMING, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.velocity_limit_servo);
	EEPROM_Write(DEF_ADDR_VELOCITY_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.torque_limit_servo);
	EEPROM_Write(DEF_ADDR_TORQUE_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, factoryDefaults.acceleration_limit_servo);
	EEPROM_Write(DEF_ADDR_ACCEL_LIMIT_SERVO, temp_bytes, 4);

	uInt2Bytes(temp_bytes, FIRMWARE_VERSION_NUMBER);
	EEPROM_Write(DEF_ADDR_VERSION_NUMBER, temp_bytes, 4);
}
//---------------------------------------------------------------------------------------------------------
