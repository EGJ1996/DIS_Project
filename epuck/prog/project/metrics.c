/* Claculates all the metrics for the epucks

*/
#include "metrics.h"
#include <math.h>


//Orientation
double orientation(int nbRobots, double* headingRobots){
	double sum = 0;
	int i ;
	for(i = 0; i < nbRobots; i++){
		sum += exp(headingRobots[i]);
	}
	return sum/nbRobots;
}


//Cohesion
double cohesion(int nbRobots, double** positionRobots){
	//get the center of mass
	//1/(1+1/N sum dist(x-xbar))
	double sum = 0;
	int i ;
	for(i = 0; i < nbRobots; i++){

	}
	double val = 1/(1+sum/nbRobots);

	return val;
}

//Velocities
double velocities(int nbRobots, double* centerOfMass){
	static double* lastCenterOfMass = 0;

}


//Performance
double performanceInstant(void){
	return 0;//orientation()*cohesion()*velocities();
}

double performanceGlobal(void){
	static double timeSteps = 0;
	static double performance = 0;
	performance += performanceInstant();
	timeSteps ++;

	return performance/timeSteps;
}