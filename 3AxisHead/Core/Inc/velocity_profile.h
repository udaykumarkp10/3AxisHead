/*
 * velocity_profile.h
 *
 *  Created on: Oct 17, 2024
 *      Author: Rajdeep
 */

#ifndef INC_VELOCITY_PROFILE_H_
#define INC_VELOCITY_PROFILE_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

typedef struct{
	double initial_pos;
	double setpoint_pos;
	double rampup_end;
	double rampdown_start;

	double derived_speed_max;
	double absolute_speed_max;

	double accleration_theta;
	double start_speed;
}Velocity_Profile;

void InitializeVelocityProfile(Velocity_Profile *velocityProfile, double abs_speed_max, double acceleration, double startup_speed);
void StartVelocityProfile(Velocity_Profile *velocityProfile, double initialPos, double setpointPos);
double ComputeProfileSpeed(Velocity_Profile *velocityProfile, double currentPos);
void SetVelocityProfileZero(Velocity_Profile *velocityProfile);
double getSign(double number);

#endif /* INC_VELOCITY_PROFILE_H_ */
