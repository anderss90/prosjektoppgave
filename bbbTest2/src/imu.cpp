/*
 * imu.c
 *
 *  Created on: Oct 1, 2014
 *      Author: anderss90
 */
#define imu_debug 0

#include "common_includes.h"
#include "imu.h"

#define IMU_ADDR 0x68

int16_t ax,ay,az,gx,gy,gz=0;
float pitch, roll, yaw, ax_mpss,ay_mpss, az_mpss, gx_ds,gy_ds,gz_ds,gx_rs,gy_rs,gz_rs =0;
float gravity_acc=9.81;
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float integralFBx = 0.0,  integralFBy = 0.0, integralFBz = 0.0;

//mahoney
float update_freq=50.0f; //50 Hz

MPU6050 my_imu(IMU_ADDR);

int imu_init(int update_frequency){
		my_imu.initialize();
		if (update_frequency!=0)update_freq=update_frequency;
		ax=0;
		ay=0;
		az=0;
		gx=0;
		gy=0;
		gz=0;
		pitch=0;
		roll=0;
		yaw=0;
		ax_mpss=0;
		ay_mpss=0;
		az_mpss=0;
		return SUCCESS;
}

float rad_to_deg(float rad){
	return (rad/3.14)*180;
}

float deg_to_rad(float deg){
	return (deg/180)*3.14;
}

//simple return globals
int imu_get_attitude_deg(float *in_pitch, float *in_roll, float *in_yaw){
	*in_pitch=pitch;
	*in_roll=roll;
	*in_yaw=yaw;
	return SUCCESS;
}
int imu_get_attitude_rad(float *in_pitch, float *in_roll, float *in_yaw){
	*in_pitch=deg_to_rad(pitch);
	*in_roll=deg_to_rad(roll);
	*in_yaw=deg_to_rad(yaw);
	return SUCCESS;
}

int imu_get_acc_mpss(float *in_ax_mpss,float *in_ay_mpss,float *in_az_mpss){
	*in_ax_mpss=ax_mpss;
	*in_ay_mpss=ay_mpss;
	*in_az_mpss=az_mpss;
	return SUCCESS;
}

int imu_get_gyro_radps(attitude_rad_t *gyro_values){
	attitude_rad_t temp;
	temp.pitch=gy_rs;
	temp.roll=gx_rs;
	temp.yaw=gz_rs;
	memcpy(gyro_values,&temp,sizeof(attitude_rad_t));
	return SUCCESS;
}

int imu_get_gyro_degps(attitude_deg_t *gyro_values){
	attitude_deg_t temp;
	temp.pitch=gy_ds;
	temp.roll=gx_ds;
	temp.yaw=gz_ds;
	memcpy(gyro_values,&temp,sizeof(attitude_deg_t));
	return SUCCESS;
}

//Get raw data and calculate values using mahoney filter
int imu_update(){

	float norm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	if (imu_debug)printf("Getting acc\n");
	my_imu.getAcceleration(&ax,&ay,&az);
	if (imu_debug)printf("Getting gyro\n");
	my_imu.getRotation(&gx,&gy,&gz);

	//converting to m/s²  and deg/sec
	ax_mpss = (float)(gravity_acc*(ax)/MPU6050_AXGAIN)-MPU6050_AXOFFSET;
	ay_mpss = (float)(gravity_acc*(ay)/MPU6050_AYGAIN)-MPU6050_AYOFFSET;
	az_mpss= (float)(gravity_acc*(az)/MPU6050_AZGAIN)-MPU6050_AZOFFSET;
	gx_rs = (float)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN*0.01745329; //degree to radians
	gy_rs = (float)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN*0.01745329; //degree to radians
	gz_rs = (float)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN*0.01745329; //degree to radians
	gx_ds = (float)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN;
	gy_ds = (float)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
	gz_ds = (float)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN;

	//printf("ax %i\t ay %i\t az %i\t gx %i\t gy %i\t gz %i\n",ax,ay,az,gx,gy,gz);
	//printf("ax_mpss %f\t ay_mpss %f\t az_mpss %f\t gx_rs %i\t gy_rs %i\t gz_rs %i\n",ax_mpss,ay_mpss,az_mpss,(gx-MPU6050_GXOFFSET),(gy-MPU6050_GYOFFSET),(gz-MPU6050_GZOFFSET));

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax_mpss == 0.0) && (ay_mpss == 0.0) && (az_mpss == 0.0))) {

		// Normalise accelerometer measurement
		norm = sqrt(ax_mpss * ax_mpss + ay_mpss * ay_mpss + az_mpss * az_mpss);
		float ax_temp=ax_mpss;
		float ay_temp=ay_mpss;
		float az_temp=az_mpss;
		ax_temp /= norm;
		ay_temp /= norm;
		az_temp /= norm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux (Radians)
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity (Radians)
		halfex = (ay_temp * halfvz - az_temp * halfvy);
		halfey = (az_temp * halfvx - ax_temp * halfvz);
		halfez = (ax_temp * halfvy - ay_temp * halfvx);

		// Compute and apply integral feedback if enabled
		if(mpu6050_mahonytwoKiDef > 0.0) {
			integralFBx += mpu6050_mahonytwoKiDef * halfex * (1.0 / mpu6050_mahonysampleFreq);	// integral error scaled by Ki
			integralFBy += mpu6050_mahonytwoKiDef * halfey * (1.0 / mpu6050_mahonysampleFreq);
			integralFBz += mpu6050_mahonytwoKiDef * halfez * (1.0 / mpu6050_mahonysampleFreq);
			gx_rs += integralFBx;	// apply integral feedback
			gy_rs += integralFBy;
			gz_rs += integralFBz;
		} else {
			integralFBx = 0.0;	// prevent integral windup
			integralFBy = 0.0;
			integralFBz = 0.0;
		}

		// Apply proportional feedback
		gx_rs += mpu6050_mahonytwoKpDef * halfex;
		gy_rs += mpu6050_mahonytwoKpDef * halfey;
		gz_rs += mpu6050_mahonytwoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx_rs *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));		// pre-multiply common factors
	gy_rs *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));
	gz_rs *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx_rs - qc * gy_rs - q3 * gz_rs);
	q1 += (qa * gx_rs + qc * gz_rs - q3 * gy_rs);
	q2 += (qa * gy_rs - qb * gz_rs + q3 * gx_rs);
	q3 += (qa * gz_rs + qb * gy_rs - qc * gx_rs);

	// Normalise quaternion
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;

	//calculate attitude
	pitch = -1*rad_to_deg(-asin(2*q1*q3 + 2*q0*q2)); // opposite sign
	roll = rad_to_deg(atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1));
	yaw = rad_to_deg(atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1));

	roll-=roll_strapdown_offset;

	return SUCCESS;
}
