/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions		     */
/*                                                                           */
/* Author: 	 06-Oct-15 by Ali Marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
/*****************************************************************************/


#include <stdio.h>
#include <string.h>
#include <math.h>

#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <ircom/ircom.h>
#include <btcom/btcom.h>

#include "flocking.h"




int e_puck_matrix[16] = {-10,-10,-5,0,0,5,10,10,10,10,5,0,0,-5,-10,-10}; // for obstacle avoidance
char buffer[80];
int i;


int robot_id = 1;				// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {0,-2};	        // Migration vector
char* robot_name;

float theta_robots[FLOCK_SIZE];

void set_rob_id(int id){
	robot_id = id-1;
}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}*/


/*
 * Updates robot position with wheel speeds
 */

void get_self_position(float* x, float* y){
	*x = my_position[0];
	*y = my_position[1];
}

void update_self_motion(int msl,int msr) {
	float theta = my_position[2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;

	char tmp[255];
	sprintf(tmp, "\n\r UPDATE: theta = %f, dtheta = %f\n\r", theta, dtheta);
	//btcomSendString(tmp);//*/
}

/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules(void) {

	char tmp[255];
		sprintf(tmp, "Speed before reynolds: speed[0] = %f speed[1] = %f\n\r",speed[robot_id][0], speed[robot_id][1]);
		//btcomSendString(tmp);
	    	//
		

	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};

	/* Compute averages over the whole flock */
	for(i=0;i<FLOCK_SIZE;i++){
		for (j=0;j<2;j++) {
			rel_avg_speed[j] += relative_speed[i][j];
            rel_avg_loc[j] += relative_pos[i][j];
		}
	}

	for(j=0;j<2;j++){
		//rel_avg_speed[j] /= FLOCK_SIZE;
		rel_avg_speed[j] /= FLOCK_SIZE - 1;
		//rel_avg_loc[j] /= FLOCK_SIZE ;
		rel_avg_loc[j] /= FLOCK_SIZE - 1;
		cohesion[j] = rel_avg_loc[j];/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	}

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for(k=0;k<FLOCK_SIZE;k++){
		if(k == robot_id)
			continue;

		if(pow(relative_pos[k][0],2) + pow(relative_pos[k][1],2) < RULE2_THRESHOLD){
			for (j=0;j<2;j++) {
    	   			dispersion[j] -= 1/relative_pos[k][j];
					//dispersion[j] = 1;
			}
		}
	}

	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	for (j=0;j<2;j++) {
		consistency[j] = rel_avg_speed[j];
    }

     //aggregation of all behaviors with relative influence determined by weights
	for (j=0;j<2;j++)
	{
		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
 	}
    //speed[robot_id][1] *= -1; //y axis of webots is inverted

    //move the robot according to some migration rule
    if(MIGRATORY_URGE == 0){
		speed[robot_id][LEFT] += 0.01*cos(my_position[2] + M_PI/2);
		speed[robot_id][RIGHT] += 0.01*sin(my_position[2] + M_PI/2);
    }
    else {
        speed[robot_id][LEFT] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
        speed[robot_id][RIGHT] += (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted
    }

    //char tmp[255];
	sprintf(tmp, "\n\r\n\r ID: %d, consistency: %f - %f dispersion: %f - %f, cohesion: %f - %f\n\r",robot_id, consistency[0], consistency[1],dispersion[0], dispersion[1], cohesion[0], cohesion[1]);
	//btcomSendString(tmp);//*/


		
		sprintf(tmp, "Speed after reynolds: speed[0] = %f speed[1] = %f\n\r",speed[robot_id][0], speed[robot_id][1]);
		//btcomSendString(tmp);
}

void compute_wheel_speeds(int* msl, int* msr) 
{
 // Compute wanted position from Reynold's speed and current location
 //float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
 //float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
 
 float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
 float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates
// printf("id = %d, x = %f, y = %f\n", robot_id, x, z);
 float Ku = 0.2;   // Forward control coefficient
 float Kw = 1;  // Rotational control coefficient
 float range = sqrtf(x*x + z*z);   // Distance to the wanted position
 float bearing = -atan2(x, z);   // Orientation of the wanted position
 
 // Compute forward control
 float u = Ku*range*cosf(bearing);
 // Compute rotational control
 float w = Kw*bearing;
 
 // Convert to wheel speeds!
 
 *msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
 *msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
// printf("bearing = %f, u = %f, w = %f, msl = %f, msr = %f\n", bearing, u, w, msl, msr);
 limit(msl,MAX_SPEED);
 limit(msr,MAX_SPEED);
}

/*void get_wheel_speeds(int* rightSpeed, int* leftSpeed){
	*rightSpeed = msr;
	*leftSpeed = msl;
}*/


/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/

void process_received_ping_messages(int val, double distance, double heading)
{
	int other_robot_id = val-1;

	prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
	prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

	relative_pos[other_robot_id][0] = distance*cos(heading);  // relative x pos
	relative_pos[other_robot_id][1] = -1.0 * distance*sin(heading);   // relative y pos

	relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
	relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);


	char tmp[255];
	sprintf(tmp, "\n\r\n\r ID: %d, rel_speed: %f - %f rel_pos: %f - %f\n\r",other_robot_id, relative_speed[other_robot_id][0], relative_speed[other_robot_id][1],relative_pos[other_robot_id][0],relative_pos[other_robot_id][1]);
	//btcomSendString(tmp);//*/
}





// Weights for the Braitenberg obstacle avoidance algorithm
int weightleft[8] = {-10, -10, -5, 0, 0, 5, 10, 10};
int weightright[8] = {10, 10, 5, 0, 0, -5, -10, -10};


void wait(unsigned long num) {
	while (num > 0) {num--;}
}
