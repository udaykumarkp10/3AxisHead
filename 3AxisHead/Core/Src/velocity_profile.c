/*
 * velocity_profile.c
 *
 *  Created on: Oct 17, 2024
 *      Author: Rajdeep
 */
#include "velocity_profile.h"

double getSign(double number){
	if(number < 0){
		return -1.0;
	}else{
		return 1.0;
	}
}

void SetVelocityProfileZero(Velocity_Profile *velocityProfile){
	velocityProfile->initial_pos = 0.0;
	velocityProfile->setpoint_pos = 0.0;
	velocityProfile->rampup_end = 0.0;
	velocityProfile->rampdown_start = 0.0;

	velocityProfile->derived_speed_max = 0.0;
}

void InitializeVelocityProfile(Velocity_Profile *velocityProfile, double abs_speed_max, double acceleration, double startup_speed){
	velocityProfile->absolute_speed_max = abs_speed_max; //
	velocityProfile->accleration_theta = atan(acceleration);
	velocityProfile->start_speed = startup_speed;

	velocityProfile->initial_pos = 0.0;
	velocityProfile->setpoint_pos = 0.0;
	velocityProfile->rampup_end = 0.0;
	velocityProfile->rampdown_start = 0.0;

	velocityProfile->derived_speed_max = 0.0;
}

void StartVelocityProfile(Velocity_Profile *velocityProfile, double initialPos, double setpointPos){
	velocityProfile->initial_pos = initialPos;
	velocityProfile->setpoint_pos = setpointPos;

	//finding contour max speed
	double lengthOfTravel = velocityProfile->setpoint_pos - velocityProfile->initial_pos;
	double permissibleMaxSpeed = (lengthOfTravel*0.5*tan(velocityProfile->accleration_theta));

	if(abs(permissibleMaxSpeed) > velocityProfile->absolute_speed_max){
		velocityProfile->derived_speed_max = getSign(permissibleMaxSpeed)*velocityProfile->absolute_speed_max;
	}else{
		velocityProfile->derived_speed_max = permissibleMaxSpeed;
	}

	//finding contour speed inflection points
	double deltaPos = velocityProfile->derived_speed_max / tan(velocityProfile->accleration_theta);
	velocityProfile->rampup_end = abs(deltaPos);
	velocityProfile->rampdown_start = abs(velocityProfile->setpoint_pos - deltaPos - velocityProfile->initial_pos);
	velocityProfile->setpoint_pos = abs(velocityProfile->setpoint_pos - velocityProfile->initial_pos);
}

double ComputeProfileSpeed(Velocity_Profile *velocityProfile, double currentPos){
	double speedSetPoint;

	double currentPosAdj = abs(currentPos - velocityProfile->initial_pos);

	if(currentPosAdj < velocityProfile->rampup_end){
		speedSetPoint = + (velocityProfile->start_speed)*getSign(velocityProfile->derived_speed_max) + getSign(velocityProfile->derived_speed_max)*currentPosAdj*tan(velocityProfile->accleration_theta);
	}else if(currentPosAdj < velocityProfile->rampdown_start){
		speedSetPoint = velocityProfile->derived_speed_max;
	}else if(currentPosAdj < velocityProfile->setpoint_pos){
		double distance_from_end = velocityProfile->setpoint_pos - currentPosAdj;
		speedSetPoint = - (velocityProfile->start_speed)*getSign(velocityProfile->derived_speed_max) + getSign(velocityProfile->derived_speed_max)*distance_from_end*tan(velocityProfile->accleration_theta);
	}else{
		speedSetPoint = 0.0;
	}

	if(speedSetPoint < velocityProfile->start_speed){
		speedSetPoint = 0.0;
	}
	return speedSetPoint;
}
