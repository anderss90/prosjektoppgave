//============================================================================
// Name        : bbbTest2.cpp
// Author      : Anders Strand
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================



#include "common_includes.h"
#include "imu.h"
#include "ins.h"
#include "hcsr04/hcsr04.h"
#include <pthread.h>
using namespace std;




#define N_BLINKS 10
int n_loops= 10*1000*1000;

char * LEDBrightness = "/sys/bus/platform/devices/gpio-leds.8/leds/beaglebone:green:usr3/brightness";
FILE *LEDHandle = NULL;
pthread_t ins_thread;
pthread_t log_thread;
pthread_t sonar_thread;

bool test_finished=0;
pthread_mutex_t test_finished_mutex;


void * ins_thread_function(void* arg){
	printf("Starting INS thread\n");
	ins_init();
	while (1){
		ins_update();
		usleep(1000*(1000/imu_update_frequency)); //50 hz

		//check if finished
		pthread_mutex_lock(&test_finished_mutex);
		if (test_finished){
			pthread_mutex_unlock(&test_finished_mutex);
			break;
		}
		pthread_mutex_unlock(&test_finished_mutex);
	}
	printf("Stopping INS thread\n");
	return NULL;

}

void * log_thread_function(void* arg){
	printf("Starting log thread\n");
	FILE *log = fopen("log.dat", "w");
	if(log==NULL){
		printf("failed to open log.dat");
		exit(-1);
	}
	state_t ins_state;
	//state_t sys_state;
	sonar_system_state_t sonar_sys_state;
	fprintf(log,"# LEGEND: Time_ms\t Pitch\t Roll\t yaw\t sonar_x\t sonar_y\t sonar_z\n");
	int time_ms=0;
	while(1){
		hcsr04_get_state(&sonar_sys_state);
		ins_get_state(&ins_state);
		fprintf(log,"%i\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n",time_ms,ins_state.attitude.pitch,ins_state.attitude.roll,ins_state.attitude.yaw,
				sonar_sys_state.position.x,sonar_sys_state.position.y,sonar_sys_state.position.z);
		usleep(1000*(1000/sonar_update_frequency)); //10 hz
		//check if finished
		pthread_mutex_lock(&test_finished_mutex);
		if (test_finished){
			pthread_mutex_unlock(&test_finished_mutex);
			break;
		}
		pthread_mutex_unlock(&test_finished_mutex);
		time_ms+=100;
	}
	fclose(log);
	printf("Stopping log thread\n");
	return NULL;
}

void * sonar_thread_function(void * arg){
	printf("Starting sonar thread\n");
	hcsr04_init();
	while(1){
		hcsr04_testing();
		usleep(1000*(1000/sonar_update_frequency));

		//check if finished
		pthread_mutex_lock(&test_finished_mutex);
		if (test_finished){
			pthread_mutex_unlock(&test_finished_mutex);
			break;
		}
		pthread_mutex_unlock(&test_finished_mutex);
	}
	printf("Stopping sonar thread\n");
	hcsr04_close();

	return NULL;
}

void mpu_6050_testing()
{
	float pitch, roll, yaw, ax_mpss,ay_mpss, az_mpss;
	printf("Starting mpu 6050 test\n");
	if (imu_init(imu_update_frequency)!=SUCCESS){
		printf("imu_init failed, returning\n");
		return;
	}
	printf("imu init complete");
	printf("Update interval: %f us\n",(1000*1000/imu_update_frequency));
	while(1){
		imu_update();
		imu_get_acc_mpss(&ax_mpss,&ay_mpss,&az_mpss);
		imu_get_attitude_deg(&pitch,&roll,&yaw);
		printf("ax:%.3f\t ,ay:%.3f\t  ,az:%.3f\t ,pitch:%.3f\t ,roll:%.3f\t ,yaw:%.3f \n",ax_mpss,ay_mpss,az_mpss,pitch,roll,yaw);
		usleep(1000*(1000/imu_update_frequency));
	}
}


void ins_testing(){
	state_t test_state;
	printf("Starting INS test\n");

	ins_init();
	while(1){
		ins_update();
		ins_get_state(&test_state);
		//printf("X:%.3f\t Y:%.3f\t Z:%.3f\t\n",test_state.position[X],test_state.position[Y],test_state.position[Z]);
		usleep(1000*(1000/imu_update_frequency));
	}
}



int main(int argc, char **argv){
	printf("Main\n");
	if(argc>1){
		if(strcmp(argv[1],"imu")==0){
			mpu_6050_testing();
		}
		else if (strcmp(argv[1],"ins")==0){
			ins_testing();
		}
	}
	else {
		//normal

		//wait 5 sec before starting
		printf("Starting logging in 5 seconds\n");
		usleep (5*1000000);

		if (pthread_create(&ins_thread,NULL,ins_thread_function,NULL)){
			printf("Error creating ins thread, exiting");
			exit(-1);
		}
		if (pthread_create(&sonar_thread,NULL,sonar_thread_function,NULL)){
			printf("Error creating sonar thread, exiting");
			exit(-1);
		}
		if (pthread_create(&log_thread,NULL,log_thread_function,NULL)){
			printf("Error creating log thread, exiting");
			exit(-1);
		}



		//wait for 10 sec to log
		//printf("|----------|\n");
		for (int i=0;i<10;i++){
			printf("-");
			fflush(stdout);
			usleep (LOG_TIME*1000*100);
		}
		printf("\n");

		pthread_mutex_lock(&test_finished_mutex);
		test_finished=true;
		pthread_mutex_unlock(&test_finished_mutex);

		pthread_join(ins_thread,NULL);
		pthread_join(sonar_thread,NULL);
		pthread_join(log_thread,NULL);
	}
	return 0;
}
