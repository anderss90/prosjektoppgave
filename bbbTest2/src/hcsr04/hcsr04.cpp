#include "../common_includes.h"
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#include "../imu.h"
#include "hcsr04.h"
#include "../ins.h"

//objects
sonar_system_state_t sonar_system_state;
sonar_t sonars[N_SONARS];
state_t current_state;
state_t prev_state;


unsigned int *pruData;
float sonar_offset[N_SONARS];
char *sonar_names[N_SONARS]={"front", "back", "left", "down", "up", "right"};

int hcsr04_sonar_reset(int sonar_index){
	sonars[sonar_index].avg_distance_cm=0;
	sonars[sonar_index].distance_cm=0;
	sonars[sonar_index].prev_distance_cm=0;
	sonars[sonar_index].validity=false;
	sonars[sonar_index].offset_cm=0;
	return SUCCESS;
}

int hcsr04_init(void) {

	/* Initialize the PRU */
	printf(">> Initializing PRU\n");
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();

	/* Open PRU Interrupt */
	if (prussdrv_open (PRU_EVTOUT_0)) {
		// Handle failure
		fprintf(stderr, ">> PRU open failed\n");
		return 1;
	}

	/* Get the interrupt initialized */
	prussdrv_pruintc_init(&pruss_intc_initdata);

	/* Get pointers to PRU local memory */
	void *pruDataMem;
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);
	pruData = (unsigned int *) pruDataMem;

	/* Execute code on PRU */
	printf(">> Executing HCSR-04 code\n");
	prussdrv_exec_program(0, "/root/remoteTest2/hcsr04/hcsr04.bin");

	//init sonar objects.
	for (int i=0;i<N_SONARS;i++){
		hcsr04_sonar_reset(i);
	}

	//init state objects
	memset(&current_state,0,sizeof(state_t));
	memset(&prev_state,0,sizeof(state_t));
	return SUCCESS;
}

int hcsr04_read(uint32_t* distances){
	// Wait for the PRU interrupt
	prussdrv_pru_wait_event (PRU_EVTOUT_0);
	prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

	// Print the distance received from the sonar
	// At 20 degrees in dry air the speed of sound is 342.2 cm/sec
	// so it takes 29.12 us to make 1 cm, i.e. 58.44 us for a roundtrip of 1 cm
	//printf("Sonar 1: %.2f Sonar 2: %.2f Sonar 3: %.2f Sonar 4: %.2f Sonar 5 %.2f Sonar 6 %.2f\n",(float) (pruData[0] / 58.44),(float)(pruData[1] / 58.44),(float) (pruData[2] / 58.44),(float) (pruData[3] / 58.44),(float) (pruData[4] / 58.44),(float) (pruData[5] / 58.44));
	memcpy(distances,pruData,sizeof(float)*N_SONARS);
	return SUCCESS;
}

int hcsr04_close(void){
	/* Disable PRU and close memory mapping*/
	prussdrv_pru_disable(0);
	prussdrv_exit();
	printf(">> PRU Disabled.\r\n");
	return SUCCESS;
}

int hcsr04_evaluate_sonar_validity(void){
	//yaw påvirker bare left/right/front/back
	//pitch og roll påvirke alle, bruk cos/sin til å beregne avstand innenfor +/- 10 grader?
	//tidligere målinger ubrukelige ved store yaw-utslag


	sonar_t* cur_son;
	for (int i=0;i<N_SONARS;i++){
		//update current sonar
		cur_son = &sonars[i];
		cur_son->validity=true; //init to true

		//check if value is sufficiently near average. (filter out outliers) Avg is still updated
		if (fabs((cur_son->distance_cm - cur_son->avg_distance_cm)) > sonar_max_deviation){
			//printf("distance: %.2f \t avg: %.2f\t diff: %.2f\t ", cur_son->distance_cm,cur_son->avg_distance_cm,(cur_son->distance_cm - cur_son->avg_distance_cm));
			cur_son->validity=false;
			//("not avg \t");
		}
		//printf ("%.2f\t",(float)cur_son->avg_distance_cm);
		//printf ("%s: %.2f\t %i\t",sonar_names[i],cur_son->distance_cm,abs((int)(cur_son->distance_cm - cur_son->avg_distance_cm)));


		//check IMU readings to evaluate validity of sonar readings
		//TODO call update_system_state before this.

		//check if attitude is within maximum
		if ((fabs(current_state.attitude.pitch)>sonar_max_attitude_rad) ||
			(fabs(current_state.attitude.roll)>sonar_max_attitude_rad) ||
			(fabs(current_state.attitude.yaw)>sonar_max_attitude_rad)
			)
		{
			cur_son->validity=false;
			printf("WHATT");
		}

		//check if angle velocity if within max
		if ((fabs(current_state.angle_velocity.pitch)>sonar_max_attitude_rate_rad) ||
			(fabs(current_state.angle_velocity.roll)>sonar_max_attitude_rate_rad) ||
			(fabs(current_state.angle_velocity.yaw)>sonar_max_attitude_rate_rad)
			)
		{
			cur_son->validity=false;
			printf("YO");
		}
		printf("%i\t",cur_son->validity);
	}

	printf("yaw:%.2f\t yaw_rate:%.2f\t max:%.2f\t \n",fabs(current_state.attitude.yaw),fabs(current_state.angle_velocity.yaw),sonar_max_attitude_rate_rad);
	return SUCCESS;
}


int hcsr04_evaluate_xyz(){
	//bruk vinkel på avstandsestimering


	return SUCCESS;
}

int hcsr04_update_distances(){
	uint32_t results[N_SONARS];
	hcsr04_read(results);
	sonar_t* cur_son;

	for (int i=0;i<N_SONARS;i++){
		//update current sonar
		cur_son = &sonars[i];

		//update values and convert to cm
		cur_son->prev_distance_cm=cur_son->distance_cm;
		cur_son->distance_cm=(float)(results[i]/us_to_cm);

		//check if value if within accepted value interval
		if (cur_son->distance_cm > sonar_max_value){
			cur_son->distance_cm=sonar_min_value; //this is assumed to be caused by beeing too close.
			//printf("too big");
		}
		if (cur_son->distance_cm < sonar_min_value){
			cur_son->distance_cm = sonar_min_value;
		}
		cur_son->avg_distance_cm = (cur_son->avg_distance_cm + (avg_alpha*(cur_son->distance_cm-cur_son->avg_distance_cm)));
	}
	printf("\n");

	return SUCCESS;
}

int hcsr04_reset_origo(void){
	return SUCCESS;
}

int evaluate_drift(){
	// kontinuerlig men lik bevegelse i front/back og left/right tyder på yaw drift
	return SUCCESS;
}

int hcsr04_update_system_state(){
	//TODO get state from sensor fusion, not from INS
	memcpy(&prev_state,&current_state,sizeof(state_t));
	ins_get_state(&current_state);
	return SUCCESS;
}

int hcsr04_testing(void){
	hcsr04_init();
	while(1){
		/*
		hcsr04_read(results);
		for (int i=0;i<N_SONARS;i++){
			printf("Sonar %i: %.2f ",i+1,(float)results[i]/58.44);
		}
		printf("\n");*/
		ins_update();

		hcsr04_update_distances();
		hcsr04_update_system_state();
		hcsr04_evaluate_sonar_validity();
		usleep(1000*1000*(1/sonar_update_frequency));

	}
	return SUCCESS;
}
