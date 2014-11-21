#include "../common_includes.h"
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#include "../imu.h"
#include "hcsr04.h"
#include "../ins.h"

//these are internal in the sonar system
sonar_system_state_t sonar_system_state;
sonar_system_state_t prev_sonar_system_state;
sonar_t sonars[N_SONARS];

//these are obtained from the fusion, do not write to these
state_t current_state;
state_t prev_state;

enum {FRONT,
	BACK,
	LEFT,
	DOWN,
	UP,
	RIGHT};

unsigned int *pruData;
float sonar_offset[N_SONARS];
char *sonar_names[N_SONARS]={"front", "back", "left", "down", "up", "right"};

int hcsr04_sonar_reset(int sonar_index){
	sonars[sonar_index].avg_distance_cm=0;
	sonars[sonar_index].distance_cm=0;
	sonars[sonar_index].prev_distance_cm=0;
	sonars[sonar_index].validity=false;
	sonars[sonar_index].offset_cm=0;
	sonars[sonar_index].index=sonar_index;
	return SUCCESS;
}

int hcsr04_sonar_reset_offset(sonar_t* sonar){
	//TODO this uses current state, not sonar system state.
	switch (sonar->index){
	case FRONT:
		sonar->offset_cm=-1*(sonar->distance_cm)-current_state.position.x;
		break;
	case BACK:
			sonar->offset_cm=(sonar->distance_cm)-current_state.position.x;
			break;
	case RIGHT:
			sonar->offset_cm=-1*(sonar->distance_cm)-current_state.position.y;
			break;
	case LEFT:
			sonar->offset_cm=(sonar->distance_cm)-current_state.position.y;
			break;
	case DOWN:
			sonar->offset_cm=-1*(sonar->distance_cm)-current_state.position.z;
			break;
	case UP:
			sonar->offset_cm=(sonar->distance_cm)-current_state.position.z;
			break;
	default:
		printf("ERROR. No valid index");
		return ERROR_INVALID_PARAMS;
		break;
	}

	//printf("%s\t offset:%.2f\t ",sonar_names[sonar->index],sonar->offset_cm);
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
		//cur_son->validity=true; //init to true
		bool temp_validity=true;

		//check if value is sufficiently near average. (filter out outliers) Avg is still updated
		if (fabs((cur_son->distance_cm - cur_son->avg_distance_cm)) > sonar_max_deviation){
			//printf("distance: %.2f \t avg: %.2f\t diff: %.2f\t ", cur_son->distance_cm,cur_son->avg_distance_cm,(cur_son->distance_cm - cur_son->avg_distance_cm));
			//cur_son->validity=false;
			temp_validity=false;
			//("not avg \t");
		}
		//printf ("%.2f\t",(float)cur_son->avg_distance_cm);
		//printf ("%s: %.2f\t %i\t",sonar_names[i],cur_son->distance_cm,abs((int)(cur_son->distance_cm - cur_son->avg_distance_cm)));


		//check IMU readings to evaluate validity of sonar readings
		//TODO call update_system_state before this.

		//check if attitude is within maximum
		if ((fabs(current_state.attitude.pitch)>sonar_max_attitude_rad) ||
			(fabs(current_state.attitude.roll)>sonar_max_attitude_rad)
			/* || /*TODO commented until total system gives feedback to yaw drift (fabs(current_state.attitude.yaw)>sonar_max_attitude_rad)*/
			)
		{
			//cur_son->validity=false;
			temp_validity=false;
			printf("WHATT");
			printf ("pitch: %.2f\t roll: %.2f\t",current_state.attitude.pitch,current_state.attitude.roll);
		}

		//check if angle velocity if within max
		if ((fabs(current_state.angle_velocity.pitch)>sonar_max_attitude_rate_rad) ||
			(fabs(current_state.angle_velocity.roll)>sonar_max_attitude_rate_rad)
			||(fabs(current_state.angle_velocity.yaw)>sonar_max_attitude_rate_rad)
			)
		{
			//cur_son->validity=false;
			temp_validity=false;
			printf("YO");
		}
		//printf("%i\t",cur_son->validity);

		//detect transitions from false to true, and reset sonar offset to match current position
		if (cur_son->validity==false){
			if (temp_validity==true){
				//offset must be reset to diff between position and measurement
				hcsr04_sonar_reset_offset(cur_son);
			}
		}
		//update validity
		cur_son->validity=temp_validity;

	}

	//printf("yaw:%.2f\t yaw_rate:%.2f\t max:%.2f\t \n",fabs(current_state.attitude.yaw),fabs(current_state.angle_velocity.yaw),sonar_max_attitude_rate_rad);
	return SUCCESS;
}

int hcsr04_evaluate_sonar_pair(sonar_pair_t* s_pair){
	sonar_t* sonar_a=s_pair->sonar_a;
	sonar_t* sonar_b=s_pair->sonar_b;
	attitude_rad_t* attitude = &current_state.attitude;
	float a_distance=sonar_a->distance_cm*cos(attitude->pitch)*cos(attitude->roll);
	float b_distance=sonar_b->distance_cm*cos(attitude->pitch)*cos(attitude->roll);
	float a_pos;
	float b_pos;
	bool a_valid=s_pair->sonar_a->validity;
	bool b_valid=s_pair->sonar_b->validity;
	float a_sign;
	float b_sign;

	//generate correct signs according to NED (North East Down) standard.
	if (s_pair->axis==X){
		if (sonar_a->index==FRONT && sonar_b->index==BACK){
			a_sign=-1;
			b_sign=1;
		}
		else if (sonar_a->index==BACK && sonar_b->index==FRONT){
			a_sign=1;
			b_sign=-1;
		}
		else {
			printf("ERROR: sonar pair does not match axis");
			return ERROR_INVALID_PARAMS;
		}
	}
	else if (s_pair->axis==Y){
		if (sonar_a->index==RIGHT && sonar_b->index==LEFT){
			a_sign=-1;
			b_sign=1;
		}
		else if (sonar_a->index==LEFT && sonar_b->index==RIGHT){
			a_sign=1;
			b_sign=-1;
		}
		else {
			printf("ERROR: sonar pair does not match axis");
			return ERROR_INVALID_PARAMS;
		}
	}
	else if (s_pair->axis==Z){
			if (sonar_a->index==DOWN && sonar_b->index==UP){
				a_sign=-1;
				b_sign=1;
			}
			else if (sonar_a->index==UP && sonar_b->index==DOWN){
				a_sign=1;
				b_sign=-1;
			}
			else {
				printf("ERROR: sonar pair does not match axis");
				return ERROR_INVALID_PARAMS;
			}
		}

	//multiply correct signs with positions
	a_pos=a_pos*a_sign;
	b_pos=b_pos*b_sign;
	a_pos=a_sign*a_distance-s_pair->sonar_a->offset_cm;
	b_pos=b_sign*b_distance-s_pair->sonar_b->offset_cm;

	if (a_valid || b_valid) {
		//at least one sonar is valid
		s_pair->validity=true;
		if (a_valid && b_valid){
			//both sonar readings are valid
			if (fabs(a_pos-b_pos)<sonar_max_pair_diff_cm){
				//the two readings agree. Position estimate is the mean of the two
				s_pair->position=((a_pos+b_pos)/2);
			}
			else if (fabs(fabs(a_pos)-fabs(b_pos))<sonar_max_pair_diff_cm){
				//the two are similar, but with different signs. This may be due to yaw drift
				//s_pair->yaw_estimate=((fabs(a_pos)+fabs(b_pos))/2)*

				//this will only work if the uav is in origo, leaving it for now.
			}
			else {
				//printf("Both are valid, but difference is too big");
				s_pair->validity=false;
			}
		}
		else {
			// only one sonar is valid
			s_pair->position=(a_valid*a_pos+b_valid*b_pos);
		}
		//printf("setting axis validity true");
	}
	else {
		//no sonars are valid -> pair is invalid
		s_pair->validity=false;
		//printf("axis invalid. a_valid: %i\t b_valid:%i\t",a_valid,b_valid);
	}

	if (s_pair->axis==Y){
		printf("a_d:%.2f\t a_os:%.2f\t a_pos:%.2f\t b_d:%.2f\t b_os:%.2f\t b_pos:%.2f\t  a_val:%i b_val:%i y_pos:%.2f\t",
				a_distance,sonar_a->offset_cm,a_pos,b_distance,sonar_b->offset_cm,b_pos,a_valid,b_valid,s_pair->position);
	}

	return SUCCESS;
}

int hcsr04_evaluate_xyz(void){
	//store previous sonar system state
	memcpy(&prev_sonar_system_state,&sonar_system_state,sizeof(sonar_system_state_t));

	//set up sonar pairs
	sonar_pair_t x_axis,y_axis,z_axis;
	memset(&x_axis,0,sizeof(sonar_pair_t));
	memset(&y_axis,0,sizeof(sonar_pair_t));
	memset(&z_axis,0,sizeof(sonar_pair_t));
	x_axis.axis=X;
	x_axis.sonar_a=&sonars[FRONT];
	x_axis.sonar_b=&sonars[BACK];
	y_axis.axis=Y;
	y_axis.sonar_a=&sonars[RIGHT];
	y_axis.sonar_b=&sonars[LEFT];
	z_axis.axis=Z;
	z_axis.sonar_a=&sonars[DOWN];
	z_axis.sonar_b=&sonars[UP];

	//evaluate pairs and estimate position
	hcsr04_evaluate_sonar_pair(&x_axis);
	hcsr04_evaluate_sonar_pair(&y_axis);
	hcsr04_evaluate_sonar_pair(&z_axis);

	//store position estimates
	if (x_axis.validity)sonar_system_state.position.x=x_axis.position;
	if (y_axis.validity)sonar_system_state.position.y=y_axis.position;
	if (z_axis.validity)sonar_system_state.position.z=z_axis.position;
	//printf("x_val:%i\t y_val:%i\t z_val:%i\t",x_axis.validity,y_axis.validity,z_axis.validity);
	//printf("y_pos:%.2f\t ",y_axis.position);
	//store validity
	sonar_system_state.pos_validity.x=x_axis.validity;
	sonar_system_state.pos_validity.y=y_axis.validity;
	sonar_system_state.pos_validity.z=z_axis.validity;

	//estimate x velocity
	//TODO implement y and z
	if (prev_sonar_system_state.pos_validity.x && sonar_system_state.pos_validity.x){
		float vel_estimate_x= (sonar_system_state.position.x-prev_sonar_system_state.position.x)/sonar_update_frequency;
		if (prev_sonar_system_state.vel_validity.x){
			//basing new velocity estimate on previous velocity using exponential average
			sonar_system_state.velocity.x=prev_sonar_system_state.velocity.x+velocity_avg_alpha*(vel_estimate_x-prev_sonar_system_state.velocity.x);
		}
		else {
			sonar_system_state.velocity.x=vel_estimate_x;
		}
		sonar_system_state.vel_validity.x=true;
	}
	else {
		sonar_system_state.vel_validity.x=false;
	}

	//printf("x:%.2f\t y:%.2f\t z:%.2f\t x_dot:%.2f\t",sonar_system_state.position.x,sonar_system_state.position.y,sonar_system_state.position.z,sonar_system_state.velocity.x);


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
	current_state.position=sonar_system_state.position;
	current_state.velocity=sonar_system_state.velocity;
	return SUCCESS;
}

void debug_print(void){
	//printf("x:%.2f\t y:%.2f\t z:%.2f\t x_dot:%.2f\t",sonar_system_state.position.x,sonar_system_state.position.y,sonar_system_state.position.z,sonar_system_state.velocity.x);
	//y test
	//printf("right:%.2f\t right_valid:%i\t  left: %.2f\t left_valid:%i\t y: %.2f\t y_valid: %i\t ",sonars[RIGHT].distance_cm,sonars[RIGHT].validity,sonars[LEFT].distance_cm,sonars[LEFT].validity,sonar_system_state.position.y, sonar_system_state.pos_validity.y);
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
		//doing this in separate thread ins_update();

		hcsr04_update_distances();
		hcsr04_update_system_state();
		hcsr04_evaluate_sonar_validity();
		hcsr04_evaluate_xyz();
		printf("\n");
		usleep(1000*1000*(1/sonar_update_frequency));

		debug_print();

	}
	return SUCCESS;
}
