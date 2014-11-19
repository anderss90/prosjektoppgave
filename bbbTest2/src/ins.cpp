/*
 * ins.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: anderss90
 */
#include "ins.h"
#include "imu.h"
#include "common_includes.h"

#define g_n -9.81f

state_t m_state;

int ins_init(){

	ins_reset();
	m_state.type=INS;

	imu_init(imu_update_frequency);
	return SUCCESS;
}

int ins_reset(){

	//can be replaced by memset?
	m_state.position.x=0;
	m_state.position.y=0;
	m_state.position.z=0;

	m_state.velocity.x=0;
	m_state.velocity.y=0;
	m_state.velocity.z=0;

	m_state.attitude.pitch=0;
	m_state.attitude.roll=0;
	m_state.attitude.yaw=0;

	m_state.angle_velocity.pitch=0;
	m_state.angle_velocity.roll=0;
	m_state.angle_velocity.yaw=0;

	return SUCCESS;
}

int ins_update(){
	float pitch, roll, yaw, ax_mpss,ay_mpss, az_mpss,v_xdot,v_ydot,v_zdot,phi,theta,psi;
	attitude_rad_t gyro_reading_rad;
	imu_update();
	imu_get_acc_mpss(&ax_mpss,&ay_mpss,&az_mpss);
	imu_get_gyro_radps(&gyro_reading_rad); //radians
	imu_get_attitude_rad(&pitch,&roll,&yaw); //radians
	phi=roll;
	theta=pitch;
	psi=yaw;

	//printf("ax:%.3f\t ,ay:%.3f\t  ,az:%.3f\t ,pitch:%.3f\t ,roll:%.3f\t ,yaw:%.3f \n",ax_mpss,ay_mpss,az_mpss,pitch,roll,yaw);

	float a_b[3]={ax_mpss,ay_mpss,az_mpss};

	//calculate velocities in NED frame using a rotation matrix (TODO fix?)
	v_xdot=ax_mpss*(cos(psi)*cos(theta))+ay_mpss*(-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi))+az_mpss*(sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi));
	v_ydot=ax_mpss*(sin(psi)*cos(theta))+ay_mpss*(cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi))+az_mpss*(-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi));
	v_zdot=ax_mpss*(-sin(theta))+ay_mpss*(cos(theta)*sin(phi))+az_mpss*(cos(theta)*cos(phi))+g_n;

	//printf("phi %.2f\t theta %.2f\t psi %.2f\t v_xdot %.2f\t v_ydot %.2f\t v_zdot %.2f\t a_x %.2f\t a_y %.2f\t a_z %.2f\t\n",phi,theta,psi,v_xdot,v_ydot,v_zdot,ax_mpss,ay_mpss,az_mpss);

	float dt = (1.0f / ins_update_frequency);

	//integrate acc to vel
	m_state.velocity.x+=v_xdot*dt;
	m_state.velocity.y+=v_ydot*dt;
	m_state.velocity.z+=v_zdot*dt;

	//integrate vel to pos
	m_state.position.x+=m_state.velocity.x*dt;
	m_state.position.y+=m_state.velocity.y*dt;
	m_state.position.z-=m_state.velocity.y*dt; //note the minus sign. positive velocity in DOWN directions gives negative movement in Z directions

	m_state.attitude.roll=roll;
	m_state.attitude.pitch=pitch;
	m_state.attitude.yaw=yaw;

	m_state.angle_velocity.pitch=gyro_reading_rad.pitch;
	m_state.angle_velocity.roll=gyro_reading_rad.roll;
	m_state.angle_velocity.yaw=gyro_reading_rad.yaw;
	return SUCCESS;
}

int ins_get_state(state_t * returned_state){

	//*returned_state=m_state;
	memcpy(returned_state,&m_state,sizeof(state_t));
	return SUCCESS;
}

