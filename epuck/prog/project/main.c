
/*
    Copyright 2007 Alexandre Campo, Alvaro Gutierrez, Valentin Longchamp.

    This file is part of libIrcom.

    libIrcom is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    libIrcom is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libIrcom.  If not, see <http://www.gnu.org/licenses/>.
*/

// simple test :  send or receive numbers, and avoid obstacles in the same time.

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

#include "main.h"
#include "braitenberg.h"

#define M_PI 3.141569

typedef enum { false, true } bool;

float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
// group number of the robot.
int ownGroup = 0;
int ownNumber = 0;

#define wheelDia  41
#define stepPerRev  1000
#define  mmPerSteps  2.0*M_PI/stepPerRev*wheelDia/2.0
#define stepsPerMm  1/mmPerSteps

int steps2DoRight = 0;
int steps2DoLeft = 0;

#define  numRobots  3

double posX[numRobots];
double posY[numRobots];
double theta[numRobots];

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}



int main()
{
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();
    //e_init_prox();

    // wait for s to start
    //btcomWaitForCommand('s');
    //btcomSendString("==== READY - IR TESTING ====\n\n");

    e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();

    // rely on selector to define the role
    int selector = getselector();

    // activate obstacle avoidance
    
    //e_activate_agenda(flocking, 5000);

    // acting as sender
    if (selector == 1)
    {    	
    	ownGroup = 1;
    	ownNumber = 1; // leader

		sendRobotInfosBT(ownGroup,ownNumber);

	    leaderPingSlave(ownGroup);
	    doLeaderStuffLoop();
	    btcomSendString(".");
    }
    
    // acting as receiver
    else if (selector == 2)
    {
    	ownGroup = 1;
    	ownNumber = 2;
		sendRobotInfosBT(ownGroup,ownNumber);

		int i = 0;
		treatIncommingMessages();
    }
    else if (selector == 3){
    	ownGroup = 1;
    	ownNumber = 3;
		sendRobotInfosBT(ownGroup,ownNumber);
		doSlaveStuffLoop();
    }
    // Group 2
    else if (selector == 4){
    	ownGroup = 2;
    	ownNumber = 1;
		sendRobotInfosBT(ownGroup,ownNumber);
		while(1);
    }
    else if (selector == 5){
    	ownGroup = 2;
    	ownNumber = 2;
		sendRobotInfosBT(ownGroup,ownNumber);
		while(1);
    }
    else if (selector == 6){
    	ownGroup = 2;
    	ownNumber = 3;
		sendRobotInfosBT(ownGroup,ownNumber);
		while(1);
    }
    // no proper role defined blink the body leds
    else 
    {
		int i = 0;
		long int j;
		while(1)
		{
		    e_set_body_led(0);
		    
		    for(j = 0; j < 200000; j++)
			asm("nop");
		    
		    e_set_body_led(1);
		    
		    for(j = 0; j < 300000; j++)
			asm("nop");
		    
		    i++;
		    i = i%8;
		}	
    }    
    
    ircomStop();
    return 0;
}

int obstacleAvoidanceThreshold = 30.0;
int obstacleAvoidanceSpeed = 500.0;

void obstacleAvoidance()
{    
    // check if an obstacle is perceived 
    double reading = 0.0;
    int obstaclePerceived = 0;
    int i=0;
    double x = 0.0, y = 0.0;
    for (i = 0; i < 8; i++)
    {
        reading = e_get_calibrated_prox(i);
	// if signal above noise
	if(reading >= obstacleAvoidanceThreshold)
	{
	    obstaclePerceived = 1;
	    
	    // compute direction to escape
	    double signal = reading - obstacleAvoidanceThreshold;
	    x += -cos(sensorDir[i]) * signal / 8.0;
	    y += sin(sensorDir[i]) * signal / 8.0;
	}
    }
    
    // no obstacles to avoid, return immediately
    if (obstaclePerceived == 0)
    {
	// go straight forward
	// change movement direction
	e_set_speed_left(obstacleAvoidanceSpeed);
	e_set_speed_right(obstacleAvoidanceSpeed);
	// return obstaclePerceived;
	return;
    }
    
    double desiredAngle = atan2 (y, x);
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    
    // turn left
    if (desiredAngle >= 0.0)
    {
	leftSpeed  = cos(desiredAngle);
	rightSpeed = 1;
    }
    // turn right
    else
    {
	leftSpeed = 1;
	rightSpeed = cos(desiredAngle);
    }
    
    // rescale values
    leftSpeed *= obstacleAvoidanceSpeed;
    rightSpeed *= obstacleAvoidanceSpeed;
    
    // change movement direction
    e_set_speed_left(leftSpeed);
    e_set_speed_right(rightSpeed);
    
    // advertise obstacle avoidance in progress
    // return 1;
}


// group is 1 or 2
// the leader will send group*10 
// and the slave group*10+slavenumber
void leaderPingSlave(){
	ircomSend(ownGroup*10+1);	
	while (ircomSendDone() == 0); 
}

void slaveRespondsLeader(int group, int slave){
	ircomSend(group*10+slave);
	while (ircomSendDone() == 0); 
}

int isPartOfGroup(int message){
	int receivedGroup = (int) message/10;
	return ownGroup == receivedGroup;
}

void doLeaderStuffLoop(void){
	bool execute = true;
	e_activate_agenda(braitenberg, 1000); //every 500ms we do obstacle sensing/avoidance
	e_activate_agenda(leaderPingSlave, 2001); //every 500ms we do obstacle sensing/avoidance
	
	while (execute == true){
		/*leaderPingSlave(ownGroup);
		int j;
		for(j = 0; j < 200000; j++)
			asm("nop");//*/
	}

}

void doSlaveStuffLoop(void){
	bool execute = true;
	int i = 0;
	while (execute == true) {
		if(i>4000){
			char tmp[128];
			sprintf(tmp, "===================== Position ==================\n\r Position X: %d\n\r Position Y: %d\n\r", posX[ownNumber], posY[ownNumber]);
			btcomSendString(tmp);
			i = 0;
		}
		i++;
	}
}

void sendRobotInfosBT(int group, int number){
	if (number == 1){
		char tmp[128];
		sprintf(tmp, "===================== LEADER ==================\n\r Group number: %d\n\r Robot number: %d\n\r", group, number);
		btcomSendString(tmp);
	}
	else{
		char tmp[128];
		sprintf(tmp, "===================== FOLLOWER ==================\n\r Group number: %d\n\r Robot number: %d\n\r", group, number);
		btcomSendString(tmp);
	}
}

void treatIncommingMessages(){
	////while (i < 200) //for loop better
	//	{
		    // ircomListen();
		    IrcomMessage imsg;
		    ircomPopMessage(&imsg);
		    if (imsg.error == 0)
		    {
				e_set_led(1, 2);
			int val = (int) imsg.value;
		    
			/* Send Value*/		
			/*char tmp[128];
			sprintf(tmp, "Receive successful : %d  - distance=%f \t direction=%f \n\r", val, (double)imsg.distance, (double)imsg.direction);
			btcomSendString(tmp);*/
			updateRobotsPosition(val, (double)imsg.distance, (double)imsg.direction);
		    }
		    else if (imsg.error > 0)
		    {
			//btcomSendString("Receive failed \n\r");		
		    }
		    // else imsg.error == -1 -> no message available in the queue

		    //if (imsg.error != -1) i++;
		//}
}

void updateRobotsPosition(int val, double distance, double heading){
	int robotNumber = (int)val - (int)val/10;

	// the angle is rotated 90°
	posX[robotNumber-1] = cos(heading-M_PI/2.0)*distance;
	posY[robotNumber-1] = sin(heading-M_PI/2.0)*distance;
}