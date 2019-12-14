#ifndef _FLOCKING_H
#define _FLOCKING_H

#define NB_SENSORS	 		8	  // Number of distance sensors
#define MAX_SPEED        	500     // Maximum speed
#define MAX_SPEED_WEB      	6.28    // Maximum speed webots
#define FLOCK_SIZE	  		2	  // Size of flock
#define M_PI              	3.141593

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T				0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.10   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (1.5/10)	// Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.07  		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.0/10)	// Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   	// Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.1/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 		1 			// Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

#define RIGHT 1
#define LEFT 0


void process_received_ping_messages(int val, double distance, double heading);
void send_ping(void);
void reynolds_rules();
void update_self_motion();
void limit(int *number, int limit);
void compute_wheel_speeds(int *msl, int *msr);
void set_rob_id(int id);


#endif
