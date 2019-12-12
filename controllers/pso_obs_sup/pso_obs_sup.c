#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "pso.h"
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define ROBOTS 5
#define MAX_ROB 5
#define ROB_RAD 0.035
#define ARENA_SIZE 1.89

#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
#define SWARMSIZE 10                    // Number of particles in swarm
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 20.0                       // Maximum velocity particle can attain
#define MININIT -20.0                   // Lower bound on initialization value
#define MAXINIT 20.0                    // Upper bound on initialization value
#define ITS 5                     // Number of iterations to run
#define DATASIZE 2*NB_SENSOR+6          // Number of elements in particle (2 Neurons with 8 proximity sensors 
                                        // + 2 recursive/lateral conenctions + 1 bias)

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2
#define DATASIZE1    6
/* Fitness definitions */
#define FIT_ITS 5 // Number of fitness steps to run during evolution

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define TIME_STEP    64

#define MAX_DIST 3.0
#define SENSOR_RANGE 4096

enum {POS_X = 0,POS_Y,POS_Z};


static WbNodeRef robs[MAX_ROB];
static WbDeviceTag emitter[MAX_ROB];
static WbDeviceTag rec[MAX_ROB];
static WbNodeRef robs[ROBOTS];		// Robots nodes
static WbFieldRef robs_trans[ROBOTS];	// Robots translation fields
static WbFieldRef robs_rotation[ROBOTS];	// Robots rotation fields
double *loc[MAX_ROB];
const double *rot[MAX_ROB];
double new_loc[MAX_ROB][3];
double new_rot[MAX_ROB][4];
double *weights;                         // Optimized result
int i,j,k;

double bestw[DATASIZE1];
int offset;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation
int t;


void calc_fitness(double[][DATASIZE],double[],int,int);
void calc_fitness1(double[][DATASIZE1],double[],int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);



/* RESET - Get device handles and starting locations */

void reset(void) {
  char rob[] = "rob0";
  char em[] = "emitter0";
  char receive[] = "receiver0";
  int i; //counter

  //For robot numbers < 10
  for (i=0;i<MAX_ROB;i++) {
    // printf("Entered reset function\n");
    robs[i] = wb_supervisor_node_get_from_def(rob);
    loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
    new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
    rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
    new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
    emitter[i] = wb_robot_get_device(em);
    if (emitter[i]==0) printf("missing emitter %d\n",i);
    rec[i] = wb_robot_get_device(receive);
    robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
    robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    rob[3]++;
    em[7]++;
    receive[8]++;
    }
    printf("Loop done\n");

  } 



void pso_braiten(){
  
  double buffer[255];
  double fit; 				// Fitness of the current FINALRUN
  double endfit; 			// Best fitness over 10 runs
  double fitvals[FINALRUNS]; 		// Fitness values for final tests
  double w[MAX_ROB][DATASIZE]; 		// Weights to be send to robots (determined by pso() )
  double f[MAX_ROB];			// Evaluated fitness (modified by calc_fitness() )
  double bestfit, bestw[DATASIZE];	// Best fitness and weights
  
  endfit = 0.0;
  bestfit = 0.0;
  for (j=0;j<1;j++) {

    // Get result of optimization
    int optimization_type = 0; // type = 0 for braitenberg weights
    weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS,optimization_type);

    // Set robot weights to optimization results
    fit = 0.0;
    for (i=0;i<MAX_ROB;i++) {
      for (k=0;k<DATASIZE;k++)
        w[i][k] = weights[k];
    }

    // Run FINALRUN tests and calculate average
    for (i=0;i<FINALRUNS;i+=MAX_ROB) {
      printf("Inside finalruns loop iteration number %d\n",i);
      calc_fitness(w,f,FIT_ITS,MAX_ROB);
      printf("Checkpoint 1\n");
      for (k=0;k<MAX_ROB && i+k<FINALRUNS;k++) {
        fitvals[i+k] = f[k];
        fit += f[k];
      }
    }
    fit /= FINALRUNS;

    // Check for new best fiitness
    if (fit > bestfit) {
      bestfit = fit;
      for (i = 0; i < DATASIZE; i++)
	       bestw[i] = weights[i];
    }

    printf("Performance: %.3f\n",fit);
    endfit += fit/10; // average over the 10 runs
  }
  printf("Average performance: %.3f\n",endfit);

  /* Send best controller to robots */
  
  // state id for pso_braiten
  
  // Signals end of the pso to the child controller
  
  printf("Printing the best weights for pso_braiten: \n");
  
  buffer[0] = -1.0;
  for (j=1;j<=DATASIZE;j++) {
    buffer[j] = bestw[j-1];
    printf("weight[%d] = %lf\n",j-1,bestw[j-1]);

  }
  buffer[DATASIZE+1] = 1000000;
  
  // Cannot seem to send the final weights to the children  
  for (i=0;i<ROBOTS;i++) {
    printf("Sending data through emitter\n");
    wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+2)*sizeof(double));
  }
  
  printf("Finished executing pso_braiten of supervisor\n");
}


double robdist(int i, int j) {
  return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
}


void send_pso_reynolds(int migrx,int migrz,double buffer[DATASIZE1+2]){

       printf("Inside braiten_reynolds of supervisor\n");
       double outbuffer[255];

       orient_migr = -atan2f(migrx,migrz);
       if (orient_migr<0) {
	orient_migr+=2*M_PI; // Keep value within 0, 2pi
        }
    	       // printf("t divisible by 10\n");
        for (i=0;i<ROBOTS;i++) {
           loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
	loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
	loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

                    		// Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it                   		

            outbuffer[0] = 1.0;
            outbuffer[1] = (double)(i);
            outbuffer[2] = loc[i][0];
        	 outbuffer[3] = loc[i][1];
        	 outbuffer[4] = loc[i][2];
        	 outbuffer[5] = (double)migrx;
        	 outbuffer[6] = (double)migrz;  
        	       	 
        	 for(int k=0;k<DATASIZE;k++){
            	 outbuffer[7+k] = buffer[k+1];
        	 }
        	 outbuffer[7+DATASIZE1+1] = buffer[DATASIZE];
        	 
        		// wb_emitter_send(emitter[i],(void *)buffer,7*sizeof(double));
                  	for(int j=0;j<ROBOTS;j++){
                      		wb_emitter_send(emitter[j],(void *)outbuffer,(7+DATASIZE1+1)*sizeof(double));	
                      		             }
                  		printf("Data sent succesfully to receiver %d\n",i);
    			}

			//Compute and normalize fitness
	printf("Finished braiten_reynolds of supervisor\n");

}

void calc_fitness1(double weights[ROBOTS][DATASIZE1], double fit[ROBOTS], int its, int numRobs) {
   
   printf("Called calc_fitness1 \n");
   double buffer[255];
   double *rbuffer;
   int i,j;
   double center_pos_start[3] = {0,0,0},center_pos_end[3] = {0,0,0};
   double center_dist;
   int n_steps;
   double rob_dist;
   char label[255];
   /* Send data to robots */
   
   buffer[0] = 1.0;
   for (i=0;i<numRobs;i++) {
    random_pos(i);
    for (j=1;j<=DATASIZE1;j++) {
      printf("buffer[%d] = %f\n",j,buffer[j]);
      buffer[j] = weights[i][j-1];
    }
    buffer[DATASIZE1+1] = its;
    // printf("Before sending data to the children\n");
    // wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE1+2)*sizeof(double));
    send_pso_reynolds(migrx,migrz,buffer);
    // printf("After sending data to the children\n");
    }
    
    // Wait for response from the leader
    while (wb_receiver_get_queue_length(rec[0]) == 0){
      wb_robot_step(64);
    }
       
   
   
   // Compute new fitness based on robot's new positions
   wb_supervisor_simulation_reset_physics();
   rob_dist = 0.0;
   n_steps = 0;
   /* Initial center of mass position */
   wb_robot_step(64); // Wait one step for the robots to
   for(i=0;i<ROBOTS;i++){
     loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
     center_pos_start[POS_X] += loc[i][POS_X];
     center_pos_start[POS_Z] += loc[i][POS_Z];
    }
    center_pos_start[POS_X] /= ROBOTS;
    center_pos_start[POS_Z] /= ROBOTS;
    
   for(i=0;i<ROBOTS;i++){
     for(j=0;j<ROBOTS;j++){
       if(i==j)
         continue;
       
       rob_dist = rob_dist + robdist(i,j);
     }
   }
   n_steps++;
  for(i=0;i<ROBOTS;i++){
    loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
    center_pos_end[POS_X] += loc[i][POS_X];
    center_pos_end[POS_Z] += loc[i][POS_Z];
    }
    
   center_pos_end[POS_X] /= ROBOTS;
   center_pos_end[POS_Z] /= ROBOTS;
   
   center_dist=sqrt(pow(center_pos_end[POS_X]- center_pos_start[POS_X],2) + pow(center_pos_end[POS_Z]- center_pos_start[POS_Z],2));
   
   center_dist/=MAX_DIST;
   printf("center_dist = %f\n",center_dist);
   rob_dist/=n_steps;
   printf("rob_dist before doing the difference = %f\n",rob_dist);
   // !!! Note: Find a better scalar than 100 to divide the number below
   rob_dist = 1.0-(rob_dist-2.0*ROB_RAD)/100;
   printf("rob_dist = %f\n",rob_dist);
   
   if(rob_dist<0.0) rob_dist = 0.0;
   for(i=0;i<ROBOTS;i++){
     fit[i]=center_dist*rob_dist;
   }
   printf("fit[0] = %f\n",fit[0]);
   //printf("Particle Fitness = %.3f\n",fit[0]);
   sprintf(label,"Last fitness: %.3f\n",fit[0]);
   wb_supervisor_set_label(2,label,0.01,0.95,0.05,0xffffff,0,FONT);
}


void pso_reynolds(){
  
  double buffer[255];
  double fit; 				// Fitness of the current FINALRUN
  double endfit; 			// Best fitness over 10 runs
  double fitvals[FINALRUNS]; 		// Fitness values for final tests
  double w[MAX_ROB][DATASIZE1]; 		// Weights to be send to robots (determined by pso() )
  double f[MAX_ROB];			// Evaluated fitness (modified by calc_fitness() )
  double bestfit;	// Best fitness and weights
  
  endfit = 0.0;
  bestfit = 0.0;
  for (j=0;j<1;j++) {

    // Get result of optimization
    int optimization_type = 1; // type = 0 for braitenberg weights
    weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE1,ROBOTS,optimization_type);

    // Set robot weights to optimization results
    fit = 0.0;
    for (i=0;i<MAX_ROB;i++) {
      for (k=0;k<DATASIZE1;k++)
        w[i][k] = weights[k];
    }

    // Run FINALRUN tests and calculate average
    for (i=0;i<FINALRUNS;i+=MAX_ROB) {
      printf("Inside finalruns loop iteration number %d\n",i);
      calc_fitness1(w,f,FIT_ITS,MAX_ROB);
      printf("Checkpoint 1\n");
      for (k=0;k<MAX_ROB && i+k<FINALRUNS;k++) {
        fitvals[i+k] = f[k];
        fit += f[k];
      }
    }
    fit /= FINALRUNS;

    // Check for new best fiitness
    if (fit > bestfit) {
      bestfit = fit;
      for (i = 0; i < DATASIZE1; i++)
	       bestw[i] = weights[i];
    }

    printf("Performance: %.3f\n",fit);
    endfit += fit/10; // average over the 10 runs
  }
  printf("Average performance: %.3f\n",endfit);

  /* Send best controller to robots */
  
  // state id for pso_braiten
  
  // Signals end of the pso to the child controller
  
  printf("Printing the best weights for pso_reynolds: \n");
  buffer[0] = -1.0;
  
  for (j=1;j<=DATASIZE1;j++) {
    buffer[j] = bestw[j-1];
    printf("weight = %lf\n",bestw[j-1]);

  }
  buffer[DATASIZE1+1] = 1000000;
  
  
  for (i=0;i<ROBOTS;i++) {
    wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE1+2)*sizeof(double));
    }
}


void braiten_reynolds(int migrx,int migrz){

       printf("Inside braiten_reynolds of supervisor\n");
       double buffer[255];

       orient_migr = -atan2f(migrx,migrz);
       if (orient_migr<0) {
	orient_migr+=2*M_PI; // Keep value within 0, 2pi
        }
        for(;;) {
	wb_robot_step(TIME_STEP);
	if (t % 10 == 0) {
    	       // printf("t divisible by 10\n");
    	       for (i=0;i<ROBOTS;i++) {

          	                      // printf("i=%d\n",i);

			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

                    		// Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it                   		

                  		buffer[0] = 2.0;
                  		buffer[1] = (double)(i);
                  		buffer[2] = loc[i][0];
                  		buffer[3] = loc[i][1];
                  		buffer[4] = loc[i][2];
                  		buffer[5] = (double)migrx;
                  		buffer[6] = (double)migrz;
                  		// wb_emitter_send(emitter[i],(void *)buffer,7*sizeof(double));
                  		for(int j=0;j<ROBOTS;j++){
                      		      wb_emitter_send(emitter[j],(void *)buffer,7*sizeof(double));	
                      		             
                                  }
                  		printf("Data sent succesfully to receiver %d\n",i);
    			}

			//Compute and normalize fitness

		};
		t += TIME_STEP;

	}
	printf("Finished braiten_reynolds of supervisor\n");

}

void send_final_data(double buf[DATASIZE1]){
  buf[0] = 5.0;
  for (j=1;j<=DATASIZE1;j++) {
    // printf("weight = %lf\n",bestw[j-1]);
    buf[j] = bestw[j-1];
  }
  buf[DATASIZE1+1] = 1000000;
  
  // Send the best reynolds weights found from pso
  
  printf("Sending the final weights to the supervisor\n");
  for (i=0;i<ROBOTS;i++) {
    printf("Sending data through emitter %d,buf[0] = %f \n",i,buf[0]);
    wb_emitter_send(emitter[i],(void *)buf,(DATASIZE1+2)*sizeof(double));
    while(wb_receiver_get_queue_length(rec[i]) == 0);
    printf("Received confirmation from the child \n");

  }
}
/* MAIN - Distribute and test conctrollers */

enum States{PSO_BRAITEN,PSO_REYNOLDS,BRAITEN_REYNOLDS};



int main(int argc,char* args[]) {
  printf("Inside supervisor's main function\n");
  // enum States main_state = BRAITEN_REYNOLDS;
  enum States main_state = PSO_BRAITEN;

    /* Initialisation */
  wb_robot_init();
  printf("Particle Swarm Optimization Super Controller\n");
  reset();
  for (i=0;i<MAX_ROB;i++) {
    wb_receiver_enable(rec[i],32);
  }
  double buf[255];

  wb_robot_step(256);
  
  // pso_braiten();
  
  // pso_braiten();
  // printf("Finished pso braiten \n");
  // pso_reynolds();
  // printf("Finished pso reynolds \n");
  braiten_reynolds(migrx,migrz);
  // printf("Finished pso reynolds in the supervisor\n");
  
    // while (wb_receiver_get_queue_length(rec[0]) == 0){
      // wb_robot_step(64);
    // }
    
    // printf("Received confirmation from the children\n");
  // braiten_reynolds(migrx,migrz);
  return 0;
}



// Makes sure no robots are overlapping
char valid_locs(int rob_id) {
  int i;
  for (i = 0; i < MAX_ROB; i++) {
    if (rob_id == i) continue;
    if (pow(new_loc[i][0]-new_loc[rob_id][0],2) + 
	      pow(new_loc[i][2]-new_loc[rob_id][2],2) < (2*ROB_RAD+0.01)*(2*ROB_RAD+0.01))
      return 0;
  }
  return 1;
}

// Randomly position specified robot
void random_pos(int rob_id) {
  //printf("Setting random position for %d\n",rob_id);
  new_rot[rob_id][0] = 0.0;
  new_rot[rob_id][1] = 1.0;
  new_rot[rob_id][2] = 0.0;
  new_rot[rob_id][3] = 2.0*3.14159*rnd();

  do {
    new_loc[rob_id][0] = ARENA_SIZE*rnd() - ARENA_SIZE/2.0;
    new_loc[rob_id][2] = ARENA_SIZE*rnd() - ARENA_SIZE/2.0;
    //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
  } while (!valid_locs(rob_id));

  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), new_loc[rob_id]);
  wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), new_rot[rob_id]);
}

// Distribute fitness functions among robots

void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
  double *rbuffer;
  double buffer1[255];
  int i,j;
  
  /* Send data to robots */

  buffer1[0] = 0.0;
  for (i=0;i<numRobs;i++) {
    random_pos(i);
    for (j=1;j<=DATASIZE;j++) {
      buffer1[j] = weights[i][j-1];
    }
    buffer1[DATASIZE+1] = its;
    // printf("Before sending data to the children\n");
    wb_emitter_send(emitter[i],(void *)buffer1,(DATASIZE+2)*sizeof(double));
    // printf("After sending data to the children\n");
  }

  /* Wait for response */
  while (wb_receiver_get_queue_length(rec[0]) == 0)
    wb_robot_step(64);

  /* Get fitness values */
  for (i=0;i<numRobs;i++) {
    // printf("Received data from robot %d\n",i);
    rbuffer = (double *)wb_receiver_get_data(rec[i]);
    // printf("Received data from the child\n");
    fit[i] = rbuffer[0];
    wb_receiver_next_packet(rec[i]);
  }
  printf("Finished executing calc_fitness function\n");
}

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIGHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
  calc_fitness(weights,fit,FIT_ITS,ROBOTS);
#if NEIGHBORHOOD == RAND_NB
  nRandom(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
  nClosest(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
  fixedRadius(neighbors,RADIUS);

#endif
}

void fitness1(double weights[ROBOTS][DATASIZE1], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
  calc_fitness1(weights,fit,FIT_ITS,ROBOTS);
#if NEIGHBORHOOD == RAND_NB
  nRandom(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
  nClosest(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
  fixedRadius(neighbors,RADIUS);

#endif
}


/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

  int i,j;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++) {

    /* Clear old neighbors */
    for (j = 0; j < ROBOTS; j++)
      neighbors[i][j] = 0;

    /* Set new neighbors randomly */
    for (j = 0; j < numNB; j++)
      neighbors[i][(int)(SWARMSIZE*rnd())] = 1;

  }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

  int r[numNB];
  int tempRob;
  double dist[numNB];
  double tempDist;
  int i,j,k;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++) {

    /* Clear neighbors */
    for (j = 0; j < numNB; j++)
      dist[j] = ARENA_SIZE;

    /* Find closest robots */
    for (j = 0; j < ROBOTS; j++) {

      /* Don't use self */
      if (i == j) continue;

      /* Check if smaller distance */
      if (dist[numNB-1] > robdist(i,j)) {
      	dist[numNB-1] = robdist(i,j);
      	r[numNB-1] = j;

      	/* Move new distance to proper place */
      	for (k = numNB-1; k > 0 && dist[k-1] > dist[k]; k--) {

      	  tempDist = dist[k];
      	  dist[k] = dist[k-1];
      	  dist[k-1] = tempDist;
      	  tempRob = r[k];
      	  r[k] = r[k-1];
      	  r[k-1] = tempRob;

      	}
      }

    }

    /* Update neighbor table */
    for (j = 0; j < ROBOTS; j++)
      neighbors[i][j] = 0;
    for (j = 0; j < numNB; j++)
      neighbors[i][r[j]] = 1;

  }

}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius) {

  int i,j;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++) {

    /* Find robots within range */
    for (j = 0; j < ROBOTS; j++) {

      if (i == j) continue;

      if (robdist(i,j) < radius) neighbors[i][j] = 1;
      else neighbors[i][j] = 0;

    }

  }

}

void step_rob() {
  wb_robot_step(64);
}
