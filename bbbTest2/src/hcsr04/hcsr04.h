/*
 * hcsr04.h
 *
 *  Created on: Nov 13, 2014
 *      Author: anderss90
 */

#ifndef HCSR04_H_
#define HCSR04_H_


int hcsr04_init(void);
int hcsr04_run(void);
int hcsr04_testing(void);

#define avg_alpha 0.4f
#define velocity_avg_alpha 0.6f
#define us_to_cm 58.44f

// limits
#define sonar_max_deviation 1.0f //10 cm max between avg and new measurement
#define sonar_max_value  450.0f
#define sonar_min_value  1.0f
#define sonar_max_attitude_rad 2*0.17f //~10 degrees
#define sonar_max_attitude_rate_rad 0.17f //~10 degrees/sec
#define sonar_max_pair_diff_cm 5 // 5 cm

//mapping inputs pins to logical directions




typedef struct sonar {
	float distance_cm;
	float prev_distance_cm;
	float avg_distance_cm;
	float offset_cm;
	bool validity;
	uint8_t index;
}sonar_t;

typedef struct sonar_pair_{
	uint8_t axis;
	sonar_t *sonar_a;
	sonar_t *sonar_b;
	float position;
	bool validity;
	float yaw_estimate;
}sonar_pair_t;

typedef struct sonar_system_state{
	position_t position;
	velocity_t velocity;
	validity_t pos_validity;
	validity_t vel_validity;
}sonar_system_state_t;


#endif /* HCSR04_H_ */
