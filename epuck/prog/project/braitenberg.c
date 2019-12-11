#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"





// Main program
void braitenberg() {

	int weightleft[8] = {-5, -5, -5, 0, 0, 5, 5, 5};
	int weightright[8] = {5, 5, 5, 0, 0, -5, -5, -5};
	char buffer[80];
	int leftwheel, rightwheel;
	int sensor[8], value;
	double sensorMean[8]=0;
	int numberOfSamples;
	int i,n;

	//Calibrate sensors
	sensor_calibrate();

	//Set the number of samples used to compute the average of the sensor values
	numberOfSamples=10;

	// Run the braitenberg algorithm

	// Forward speed
	leftwheel = 400;
	rightwheel = 400;
	//Compute an average value of each sensor on multiple samples to reduce noise
	for (n=0;n<numberOfSamples;n++) {
		// Get sensor values
		for (i = 0; i < 8; i++) {
			// Use the sensorzero[i] value generated in sensor_calibrate() to zero sensorvalues
			sensor[i] = e_get_prox(i)-sensorzero[i];
			//linearize the sensor output and compute the average
			sensorMean[i]+=12.1514*log((double)sensor[i])/(double)numberOfSamples;
		}
	}
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
	e_set_speed_left(leftwheel);
	e_set_speed_right(rightwheel);

	// Indicate with leds on which side we are turning (leds are great for debugging)
	if (leftwheel>rightwheel) {
		e_set_led(1, 1);
		e_set_led(7, 0);
	}
	else {
		e_set_led(1, 0);
		e_set_led(7, 1);
	}
}
