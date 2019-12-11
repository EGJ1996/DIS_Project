#ifndef _METRICS
#define _METRICS

double orientation(int nbRobots, double* headingRobots);
double cohesion(int nbRobots, double** positionRobots);
double velocities(int nbRobots, double* centerOfMass);
double performanceInstant(void);
double performanceGlobal(void);

#endif