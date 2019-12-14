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

#include "braitenberg.h"


// Main program
void braitenberg(int* bSpeeds) {

	int weightleft[8] = {-6,-3,-1.2,1,1,1.2,3,-4};
	int weightright[8] = {-4,3,1.2,1,1,-1.2,-3,-5};

	//char buffer[80];
	int leftwheel, rightwheel;
	int sensor[8];
	double sensorMean[8] = {0,0,0,0,0,0,0,0};
	double sensorSum = 0;
	int numberOfSamples;
	int i,n;

	//Set the number of samples used to compute the average of the sensor values
	numberOfSamples=1;

	// Run the braitenberg algorithm

	//Compute an average value of each sensor on multiple samples to reduce noise
	for (n=0;n<numberOfSamples;n++) {
		// Get sensor values
		for (i = 0; i < 8; i++) {
			// Use the sensorzero[i] value generated in sensor_calibrate() to zero sensorvalues
			sensor[i] = e_get_calibrated_prox(i);
			//linearize the sensor output and compute the average
			sensorMean[i]+=12.1514*log((double)sensor[i])/(double)numberOfSamples;
			sensorSum += sensor[i];
		}
	}

	if(sensorSum > 30){ // obstacle detection threshold
		// Add the weighted sensors values
		for (i = 0; i < 8; i++) {
			leftwheel += weightleft[i] * (int)sensorMean[i];
			rightwheel += weightright[i] * (int)sensorMean[i];
		}

		// Speed bounds, to avoid setting to high speeds to the motor
		if (leftwheel > 1000) {leftwheel = 1000;}
		if (rightwheel > 1000) {rightwheel = 1000;}
		if (leftwheel < -1000) {leftwheel = -1000;}
		if (rightwheel < -1000) {rightwheel = -1000;}
		//e_set_speed_left(leftwheel);
		//e_set_speed_right(rightwheel);
		bSpeeds[LEFT] = leftwheel;
		bSpeeds[RIGHT] = rightwheel;
		
	}
	else{
		bSpeeds[RIGHT] = 0;
		bSpeeds[LEFT] = 0;
	}
}
