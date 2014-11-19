/*
 * imu.h
 *
 *  Created on: Oct 1, 2014
 *      Author: anderss90
 */

#ifndef IMU_H_
#define IMU_H_

#define MPU6050_AXOFFSET 0.65f
#define MPU6050_AYOFFSET -1.1f
#define MPU6050_AZOFFSET 0.1f
#define MPU6050_AXGAIN 16384.0
#define MPU6050_AYGAIN 16384.0
#define MPU6050_AZGAIN 16384.0
#define MPU6050_GXOFFSET -3
#define MPU6050_GYOFFSET 4
#define MPU6050_GZOFFSET -22
#define MPU6050_GXGAIN 16.4
#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4

//mahoney-filter
#define mpu6050_mahonysampleFreq 50.0 // sample frequency in Hz
#define mpu6050_mahonytwoKpDef (4.0) // 2 * proportional gain
#define mpu6050_mahonytwoKiDef (0.1) // 2 * integral gain


int imu_init(int update_frequency);
int imu_get_attitude_deg(float *in_pitch, float *in_roll, float *in_yaw);
int imu_get_attitude_rad(float *in_pitch, float *in_roll, float *in_yaw);
int imu_get_acc_mpss(float *in_ax_mpss,float *in_ay_mpss,float *in_az_mpss);
int imu_get_gyro_radps(attitude_rad_t *gyro_values);
int imu_get_gyro_degps(attitude_deg_t *gyro_values);

//Get raw data and calculate values using mahoney filter
int imu_update();






#endif /* IMU_H_ */
