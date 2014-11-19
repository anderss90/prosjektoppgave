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
using namespace std;


#define N_BLINKS 10
int n_loops= 10*1000*1000;

char * LEDBrightness = "/sys/bus/platform/devices/gpio-leds.8/leds/beaglebone:green:usr3/brightness";
FILE *LEDHandle = NULL;


void led_blinking(int n_blinks);

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
		usleep((1000*1000/imu_update_frequency));
	}
}


void ins_testing(){
	state_t test_state;
	printf("Starting INS test");

	ins_init();
	while(1){
		ins_update();
		ins_get_state(&test_state);
		//printf("X:%.3f\t Y:%.3f\t Z:%.3f\t\n",test_state.position[X],test_state.position[Y],test_state.position[Z]);
		usleep(1000*(1000/imu_update_frequency));
	}
}

int main(int argc, char **argv){
	printf("Main");
	if(argc>1){
		if(strcmp(argv[1],"imu")==0){
			mpu_6050_testing();
		}
		else if (strcmp(argv[1],"ins")==0){
			ins_testing();
		}
	}
	else {
		hcsr04_testing();
	}
	return 0;
}

void led_blinking(int n_blinks){
	cout << "!!!Blinking led 3!!!" << endl; // prints !!!Hello World!!!
		for (int i =0;i<N_BLINKS;i++){
			if ((LEDHandle = fopen(LEDBrightness, "r+"))!=NULL){

				fwrite("1", sizeof(char),1,LEDHandle);
				//while(1);
				usleep(10000);
				fclose(LEDHandle);
			}
			else {
				cout << "failed to open IO, exiting" << endl;
				return;
			}
			if ((LEDHandle = fopen(LEDBrightness, "r+"))!=NULL){

				fwrite("0", sizeof(char),1,LEDHandle);

				usleep(10000);
				fclose(LEDHandle);
			}
			else {
				cout << "failed to open IO, exiting" << endl;
				return;
			}
		}
}

