/*
 * common_includes.h
 *
 *  Created on: Oct 1, 2014
 *      Author: anderss90
 */

#ifndef COMMON_INCLUDES_H_
#define COMMON_INCLUDES_H_
#include "error.h"
#include "mpu6050/MPU6050.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <cmath>
#include <string.h>

enum {X,Y,Z};
enum {PITCH,ROLL,YAW};
enum {IMU,INS,SONAR};

#define imu_update_frequency 50.0f
#define ins_update_frequency 50.0f
#define sonar_update_frequency 10.0f

#define LOG_TIME 15

//Sonars
#define N_SONARS 6

typedef struct attitude_rad{
	double pitch;
	double roll;
	double yaw;
}attitude_rad_t;

typedef struct attitude_deg{
	double pitch;
	double roll;
	double yaw;
}attitude_deg_t;

typedef struct position {
	double x;
	double y;
	double z;
}position_t;

typedef struct velocity {
	double x;
	double y;
	double z;
}velocity_t;

typedef struct validity {
	bool x;
	bool y;
	bool z;
}validity_t;

// all angles in radians
typedef struct state{
	position_t position;
	velocity_t velocity;
	attitude_rad_t attitude;
	attitude_rad_t angle_velocity;
	uint8_t type;
}state_t;





#endif /* COMMON_INCLUDES_H_ */
