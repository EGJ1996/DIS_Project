#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define MAX_SPEED 1000.0 // Maximum speed of wheels in each direction
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define MAX_ACC 1000.0 // Maximum amount speed can change in 128 ms
#define NB_SENSOR 8 // Number of proximity sensors

#define DATASIZE 2*NB_SENSOR+6 // Number of elements in particle

// Fitness definitions
#define MAX_DIFF (2*MAX_SPEED) // Maximum difference between wheel speeds
#define MAX_SENS 4096.0 // Maximum sensor value

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (2.5/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))


WbDeviceTag ds[NB_SENSOR];
WbDeviceTag emitter;
WbDeviceTag receiver;
WbDeviceTag left_motor, right_motor;
double good_w[DATASIZE] = {-11.15, -16.93, -8.20, -18.11, -17.99, 8.55, -8.89, 3.52, 29.74,
			     -7.48, 5.61, 11.16, -9.54, 4.58, 1.41, 2.09, 26.50, 23.11,
			     -3.44, -3.78, 23.20, 8.41};



int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

float loc[FLOCK_SIZE][3];	// X, Z, Theta of all robots
float prev_loc[FLOCK_SIZE][3];	// Previous X, Z, Theta values
float speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];	// != 0 if initial positions have been received
float migr[2];                   // Migratory Urge

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze


int braiten;

void reset(void) {
    char text[4];
    int i;
    text[1]='s';
    text[3]='\0';
    for (i=0;i<NB_SENSOR;i++) {
        text[0]='p';
        text[2]='0'+i;
        ds[i] = wb_robot_get_device(text); // distance sensors
    }
    emitter = wb_robot_get_device("emitter_epuck");
    receiver = wb_robot_get_device("receiver_epuck");
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    for(i=0;i<NB_SENSORS;i++)
        {
		wb_distance_sensor_enable(ds[i],64);
        }
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	char* robot_name; 
	robot_name=(char*) wb_robot_get_name();
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
  
	for(i=0; i<FLOCK_SIZE; i++) 
	{
		initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	}
  
  	printf("Reset: robot %d\n",robot_id_u);
}

// Generate random number from 0 to 1
double rnd() {
  
    return (double)rand()/RAND_MAX;
}



// Generate Gaussian random number with 0 mean and 1 std
double gauss(void) {
    double x1, x2, w;

    do {
        x1 = 2.0 * rnd() - 1.0;
        x2 = 2.0 * rnd() - 1.0;
        w = x1*x1 + x2*x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w))/w);
    return(x1*w);
}

// S-function to transform v variable to [0,1]
double s(double v) {
    if (v > 5)
        return 1.0;
    else if (v < -5)
        return 0.0;
    else
        return 1.0/(1.0 + exp(-1*v));
}

void limitf(float *number, int limit) {
	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
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

/*
 * Updates robot position with wheel speeds
 * Used for odometry
 */
void update_self_motion(int msl, int msr) {
	float theta = loc[robot_id][2];
  
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	loc[robot_id][0] += dx;
	loc[robot_id][1] += dz;
	loc[robot_id][2] += dtheta;
  
	// Keep orientation within 0, 2pi
	if (loc[robot_id][2] > 2*M_PI) loc[robot_id][2] -= 2.0*M_PI;
	if (loc[robot_id][2] < 0) loc[robot_id][2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	//float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	//float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
//	printf("id = %d, x = %f, y = %f\n", robot_id, x, z);
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
//	printf("bearing = %f, u = %f, w = %f, msl = %f, msr = %f\n", bearing, u, w, msl, msr);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}

/*
 * Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i, j, k;			// Loop counters
	float avg_loc[2] = {0,0};	// Flock average positions
	float avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	
	/* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++) {
		if (i == robot_id) 
		{	
			// don't consider yourself for the average 
			continue;
		}
          	for (j=0;j<2;j++) 
		{
			avg_speed[j] += speed[i][j];
			avg_loc[j] += loc[i][j];
		}
	}
	
        for (j=0;j<2;j++) 
	{
          	avg_speed[j] /= FLOCK_SIZE-1;
          	avg_loc[j] /= FLOCK_SIZE-1;

          	
        }
	
	/* Reynold's rules */
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	for (j=0;j<2;j++) {
		// If center of mass is too far
		if (sqrt(pow(loc[robot_id][0]-avg_loc[0],2)+pow(loc[robot_id][1]-avg_loc[1],2)) > RULE1_THRESHOLD) 
		{
         		cohesion[j] = avg_loc[j] - loc[robot_id][j];   // Relative distance to the center of the swarm
		}
	}
	
  
  
	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {        // Loop on flockmates only
			// If neighbor k is too close (Euclidean distance)
			if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD) 
			{
				for (j=0;j<2;j++) 
				{
					dispersion[j] += 1/(loc[robot_id][j] -loc[k][j]);	// Relative distance to k
				}
			}
		}
	}
  
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
        consistency[0] = 0;
        consistency[1] = 0;
        /* add code for consistency[j]*/
        for (j=0;j<2;j++) 
        {
            consistency[j] = avg_speed[j] - speed[robot_id][j];
        }
  
        // aggregation of all behaviors with relative influence determined by weights
//        printf("id = %d, cx = %f, cy = %f\n", robot_id, cohesion[0], cohesion[1]);
        if(robot_id == 0)
        printf("id = %d, %f %f, %f %f, %f %f\n", robot_id, cohesion[0], -cohesion[1], dispersion[0], -dispersion[1], consistency[0], -consistency[1]);
        
        for (j=0;j<2;j++) 
	{
                 speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
                 speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
                 speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
         }
        speed[robot_id][1] *= -1; //y axis of webots is inverted
        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          speed[robot_id][0] += 0*0.01*cos(loc[robot_id][2] + M_PI/2);
          speed[robot_id][1] += 0*0.01*sin(loc[robot_id][2] + M_PI/2);
        }
        else {
            speed[robot_id][0] += MIGRATION_WEIGHT*(migr[0]-loc[robot_id][0]);
            speed[robot_id][1] -= MIGRATION_WEIGHT*(migr[1]-loc[robot_id][1]); //y axis of webots is inverted
        }
}

/*
 * Initialize robot's position
 */
 
 // Fixed initial position, no need to set one programmatically
// void initial_pos(void){
	// char *inbuffer;	
	// int rob_nb;
	// float rob_x, rob_z, rob_theta; // Robot position and orientation


	
	// while (initialized[robot_id] == 0) {
		
		// wait for message
		// while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);
		
		// inbuffer = (char*) wb_receiver_get_data(receiver);
		// sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta, &migr[0], &migr[1]);
    
                // //robot_nb %= FLOCK_SIZE;
                // if (rob_nb == robot_id) 
		// {
			// Initialize self position
			// loc[rob_nb][0] = rob_x; 		// x-position
			// loc[rob_nb][1] = rob_z; 		// z-position
			// loc[rob_nb][2] = rob_theta; 		// theta
			// prev_loc[rob_nb][0] = loc[rob_nb][0];
			// prev_loc[rob_nb][1] = loc[rob_nb][1];
			// initialized[rob_nb] = 1; 		// initialized = true
		// }		
		// wb_receiver_next_packet(receiver);
	// }	
  // }

// Find the fitness for obstacle avoidance of the passed controller
double fitfunc_reynolds(double weights[DATASIZE],int its) {
    
    reynolds_rules();

    // Average speed
    fit_speed += (fabs(speed[robot_id][0]) + fabs(speed[robot_id][1]))/(2.0*MAX_SPEED);
    // Difference in speed
    fit_diff += fabs(speed[robot_id][0] - speed[robot_id][1])/MAX_DIFF;
    // Sensor values
    for (i=0;i<NB_SENSOR;i++) {
        sens_val[i] += ds_value[i]/MAX_SENS;
    }

    // Find most active sensor
    for (i=0;i<NB_SENSOR;i++) {
        if (sens_val[i] > fit_sens) fit_sens = sens_val[i];
    }
    // Average values over all steps
    fit_speed /= its;
    fit_diff /= its;
    fit_sens /= its;

    // Better fitness should be higher
    fitness = fit_speed*(1.0 - sqrt(fit_diff))*(1.0 - fit_sens);

    return fitness;
}

//-------------MAIN------------//


int main(){ 	

	int msl, msr;			// Wheel speeds
         /*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int rob_nb;			// Robot number
	float rob_x, rob_z, rob_theta;  // Robot position and orientation
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	double *inbuffer;			// Buffer for the receiver node
	int max_sens;			// Store highest sensor value
	
	
            wb_robot_init();
            reset();
            for(i=0;i<NB_SENSOR;i++) {
                wb_distance_sensor_enable(ds[i],64);
            }
            wb_receiver_enable(receiver,32);
            //wb_differential_wheels_enable_encoders(64);
            braiten = 0; // Don't run forever
        
            wb_robot_step(64);
 	reset();			// Resetting the robot
	
	msl = 0; msr = 0;  
	max_sens = 0;
	
	// Forever
	for(;;){
    	    
	    bmsl = 0; bmsr = 0;
                sum_sensors = 0;
                max_sens = 0;
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) 
		{
    	                 distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
                            sum_sensors += distances[i]; // Add up sensor values
                            max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value
                            
			    // Weighted sum of distance sensor values for Braitenberg vehicle
                            bmsr += e_puck_matrix[i] * distances[i];
                            bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
                }
                
		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
                bmsl+=66; bmsr+=72;
              
		/* Get information */
		
		// We need the robot's id because a robot needs the speeds of the other robots too
		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0) 
		{
    		           printf("Received data from supervisor\n");
		
			inbuffer = (double*) wb_receiver_get_data(receiver);
			// printf("Size of the buffer = %d\n",sizeof(inbuffer));
                                 printf("datasize = %d\n",DATASIZE);

			printf("Printing buffer in obs_con: \n");
			for(int k=0;k<sizeof(inbuffer);k++){
  			   printf("%lf ",inbuffer[k]);
			}
			printf("\n");
			// sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta);
			
			
			if ((int) rob_nb/FLOCK_SIZE == (int) robot_id/FLOCK_SIZE) {
			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				loc[robot_id][0] = inbuffer[DATASIZE+1]; //x-position
				loc[robot_id][1] = inbuffer[DATASIZE+2];; //z-position
				loc[robot_id][2] = inbuffer[DATASIZE+3];; //theta
				prev_loc[robot_id][0] = loc[robot_id][0];
				prev_loc[robot_id][1] = loc[robot_id][1];
				initialized[robot_id] = 1;
			} else {
				// Get position update
//				printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
				prev_loc[robot_id][0] = loc[robot_id][0];
				prev_loc[robot_id][1] = loc[robot_id][1];
				loc[robot_id][0] = inbuffer[DATASIZE+1]; //x-position
				loc[robot_id][1] = inbuffer[DATASIZE+2]; //z-position
				loc[robot_id][2] = inbuffer[DATASIZE+3]; //theta
			}
			
			speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
			speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);
			// count++;
			}
			
			wb_receiver_next_packet(receiver);
		}
		
		// Compute self position & speed
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];
		
		update_self_motion(msl,msr);
		
		speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);
    
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		//printf("%f %f\n", speed[robot_id][0], speed[robot_id][1]);
    
		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
                  
		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
                      wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
    
		// Send current position to neighbors, uncomment for I15, don't forget to uncomment the declration of "outbuffer" at the begining of this function.
		/*Implement your code here*/

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}


// int main() {
    // double buffer[255];
    // double *rbuffer;
    // double fit;
    // int i;

    // wb_robot_init();
    // reset();
    // for(i=0;i<NB_SENSOR;i++) {
        // wb_distance_sensor_enable(ds[i],64);
    // }
    // wb_receiver_enable(rec,32);
    // //wb_differential_wheels_enable_encoders(64);
    // braiten = 0; // Don't run forever

    // wb_robot_step(64);
    // while (1) {
        // Wait for data
        // while (wb_receiver_get_queue_length(rec) == 0) {
            // wb_robot_step(64);
        // }
        // rbuffer = (double *)wb_receiver_get_data(rec);

        // Check for pre-programmed avoidance behavior
        // if (rbuffer[DATASIZE] == -1.0) {
            // braiten = 1;
            // fitfunc(good_w,100);
            
            // Otherwise, run provided controller
        // } else {
            // rbuffer contains the braitenberg weights
            // fit = fitfunc(rbuffer,rbuffer[DATASIZE]);
            // buffer[0] = fit;
            // wb_emitter_send(emitter,(void *)buffer,sizeof(double));
        // }

        // wb_receiver_next_packet(rec);
    // }
        
    // return 0;
// }
