#include <stdio.h>
#include <math.h>
#include "pso.h"
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define ROBOTS 10
#define MAX_ROB 10
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
#define ITS 20                          // Number of iterations to run
#define DATASIZE 2*NB_SENSOR+6          // Number of elements in particle (2 Neurons with 8 proximity sensors 
                                        // + 2 recursive/lateral conenctions + 1 bias)

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 180                     // Number of fitness steps to run during evolution

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define ROB_NB 5

static WbNodeRef robs[ROB_NB];
WbDeviceTag emitter[ROB_NB];
WbDeviceTag rec[ROB_NB];
const double *loc[ROB_NB];
const double *rot[ROB_NB];
double new_loc[ROB_NB][3];
double new_rot[ROB_NB][4];


void calc_fitness(double[][DATASIZE],double[],int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);

/* RESET - Get device handles and starting locations */
void reset(void) {
  // Device variables
  char rob[] = "rob0";
  char em[] = "emitter0";
  char receive[] = "receiver0";
  int i; //counter

  //For robot numbers < 10
  for (i=0;i<ROB_NB;i++) {
    robs[i] = wb_supervisor_node_get_from_def(rob);
    loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
    new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
    rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
    new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
    emitter[i] = wb_robot_get_device(em);
    if (emitter[i]==0) printf("missing emitter %d\n",i);
    rec[i] = wb_robot_get_device(receive);
    rob[3]++;
    em[7]++;
    receive[8]++;
  }
  
}

/* MAIN - Distribute and test conctrollers */
int main() {
  double *weights;                         // Optimized result
  double buffer[255];			   // Buffer for emitter
  int i,j,k;			           // Counter variables

  /* Initialisation */
  wb_robot_init();
  printf("Particle Swarm Optimization Super Controller\n");
  reset();
  for (i=0;i<ROB_NB;i++) {
    wb_receiver_enable(rec[i],32);
  }

  wb_robot_step(256);

  double fit; 				// Fitness of the current FINALRUN
  double endfit; 			// Best fitness over 10 runs
  double fitvals[FINALRUNS]; 		// Fitness values for final tests
  double w[ROB_NB][DATASIZE]; 		// Weights to be send to robots (determined by pso() )
  double f[ROB_NB];			// Evaluated fitness (modified by calc_fitness() )
  double bestfit, bestw[DATASIZE];	// Best fitness and weights

  // Optimize controllers
  endfit = 0.0;
  bestfit = 0.0;
  // Do 10 runs and send the best controller found to the robot
  for (j=0;j<10;j++) {

    // Get result of optimization
    weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);

    // Set robot weights to optimization results
    fit = 0.0;
    for (i=0;i<ROB_NB;i++) {
      for (k=0;k<DATASIZE;k++)
        w[i][k] = weights[k];
    }

    // Run FINALRUN tests and calculate average
    for (i=0;i<FINALRUNS;i+=ROB_NB) {
      calc_fitness(w,f,FIT_ITS,ROB_NB);
      for (k=0;k<ROB_NB && i+k<FINALRUNS;k++) {
        fitvals[i+k] = f[k];
        fit += f[k];
      }
    }
    fit /= FINALRUNS;

    // Check for new best fitness
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
  for (j=0;j<DATASIZE;j++) {
    buffer[j] = bestw[j];
  }
  buffer[DATASIZE] = 1000000;
  for (i=0;i<ROBOTS;i++) {
    wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
  }

  /* Wait forever */
  while (1) wb_robot_step(64);

  return 0;
}

// Makes sure no robots are overlapping
char valid_locs(int rob_id) {
  int i;

  for (i = 0; i < ROB_NB; i++) {
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
  double buffer[255];
  double *rbuffer;
  int i,j;

  /* Send data to robots */
  for (i=0;i<numRobs;i++) {
    random_pos(i);
    for (j=0;j<DATASIZE;j++) {
      buffer[j] = weights[i][j];
    }
    buffer[DATASIZE] = its;
    wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
  }

  /* Wait for response */
  while (wb_receiver_get_queue_length(rec[0]) == 0)
    wb_robot_step(64);

  /* Get fitness values */
  for (i=0;i<numRobs;i++) {
    rbuffer = (double *)wb_receiver_get_data(rec[i]);
    fit[i] = rbuffer[0];
    wb_receiver_next_packet(rec[i]);
  }
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

/* Get distance between robots */
double robdist(int i, int j) {
  return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
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
