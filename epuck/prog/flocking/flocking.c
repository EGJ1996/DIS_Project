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
#include <math.h>
#include <string.h>

// #include "p30f6014a.h"
// #include "stdio.h"
// #include "string.h"

// #include "./codec/e_sound.h"
// #include "./motor_led/e_init_port.h"
// #include "./motor_led/e_led.h"
// #include "./motor_led/e_motors.h"
// #include "./uart/e_uart_char.h"
// //#include "./a_d/advance_ad_scan/e_acc.h"
// #include "./a_d/advance_ad_scan/e_prox.h"
// #include "./a_d/advance_ad_scan/e_ad_conv.h"
#include <ircom/ircom.h>


#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>
#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         1000     // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define FLOCK_SIZE	  5	  // Size of flock
#define M_PI              3.141593

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.3/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))


int e_puck_matrix[16] = {-10,-10,-5,0,0,5,10,10,10,10,5,0,0,-5,-10,-10}; // for obstacle avoidance
char buffer[80];
int i;


int robot_id;				// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {25,-25};	        // Migration vector
char* robot_name;

float theta_robots[FLOCK_SIZE];


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}


/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
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
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 1;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {
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
		rel_avg_speed[j] /= FLOCK_SIZE - 1;
		rel_avg_loc[j] /= FLOCK_SIZE - 1;
	}
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
    
        for (j=0;j<2;j++) 
	{	
            cohesion[j] = rel_avg_loc[j];
	}

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for(k=0;k<FLOCK_SIZE;k++){
		if(k == robot_id)
			continue;
		
		if(pow(relative_pos[k][0],2) + pow(relative_pos[k][1],2) < RULE2_THRESHOLD){
			for (j=0;j<2;j++) {
    	   			dispersion[j] -= 1/relative_pos[k][j];
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
        speed[robot_id][1] *= -1; //y axis of webots is inverted
        
        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
          speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
        }
        else {
            speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
            speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted
        }
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{	
	ircomSend(0+robot_id);
	while(ircomSendDone() == 0);
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/

void process_received_ping_messages(void)
{
        float message_direction;
        double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
        int other_robot_id;

	// While there are messages in the message queu	
	while(1){
		
		IrcomMessage imsg;
		ircomPopMessage(&imsg);	
		if(imsg.error > 0){
			// pop returned an error, hence there are no more messages in the queue(break out of the while loop)
			break;
		}
		message_direction = imsg.direction; 
		message_rssi = imsg.distance;

	        theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));
		

		other_robot_id = (int)imsg.value; // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos

		printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],message_direction,my_position[2]*180.0/3.141592);
		
		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);
	}	
		 
}


// the main function
int main(){ 

		
	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int max_sens;	

	int sensor[NB_SENSORS];
	int value;	
 	// reset();			// Resetting the robot

	
	e_init_port();
    	e_init_ad_scan();
    	e_init_uart1();
    	e_led_clear();	
    	e_init_motors();
    	e_start_agendas_processing();


    	e_calibrate_ir(); 

    	ircomStart();
	ircomEnableContinuousListening();
    	ircomListen();
	msl = 0; msr = 0; 
	max_sens = 0; 
	
	robot_id = getselector();
	// Forever
	for(;;){
		

		bmsl = 0; bmsr = 0;
                sum_sensors = 0;
		max_sens = 0;
                
		/* Braitenberg */
		
		for(i=0;i<NB_SENSORS;i++){
			sensor[i] = e_get_prox(i);
		}
		
		for(i=0;i<NB_SENSORS;i++) 
		{	    
			    value = (sensor[i]>>4);
                            max_sens = max_sens>sensor[i]?max_sens:sensor[i]; // Check if new highest sensor value

                            // Weighted sum of distance sensor values for Braitenburg vehicle
			    bmsr += e_puck_matrix[i] * value; 
                            bmsl += e_puck_matrix[i+NB_SENSORS] * value; 
                 }

		 // Adapt Braitenberg values (empirical tests)
                 bmsl/=MIN_SENS; bmsr/=MIN_SENS;
                 bmsl+=66; bmsr+=72;
              
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl,msr);
		
		process_received_ping_messages();

		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);
    
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
 		e_set_speed_left(msl);
		e_set_speed_right(msr);		
    
	}
}  
  

