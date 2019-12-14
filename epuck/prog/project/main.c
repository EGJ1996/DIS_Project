
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
#include "flocking.h"

#define M_PI 3.141593

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

#define  NB_ROBOTS_PER_GROUP  3

double posX[NB_ROBOTS_PER_GROUP] = {0,0,0};
double posY[NB_ROBOTS_PER_GROUP] = {0,0,0};
double theta[NB_ROBOTS_PER_GROUP] = {0,0,0};

int bSpeeds[2] = {0,0};
int rSpeeds[2] = {0,0};
int msl = 0,msr = 0;


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

    msl = 0;
    msr = 0;
    int i = 0;
    if (selector == 1)
    {    	
    	ownGroup = 1;
    	ownNumber = 1; // leader
    	set_rob_id(ownNumber);

    	for(i = 0; i < 8; i+=2)
    		e_set_led(i+1,1);

	    sendPing();
	    doLeaderStuffLoop();
	    btcomSendString(".");
    }
    
    // acting as receiver
    else if (selector == 2)
    {
    	ownGroup = 1;
    	ownNumber = 2;
    	set_rob_id(ownNumber);
    	for(i = 0; i < 8; i+=2)
    		e_set_led(i+1,1);
		
	    sendPing();
	    doLeaderStuffLoop();
	    btcomSendString(".");
    }
    else if (selector == 3){
    	ownGroup = 1;
    	ownNumber = 3;
    	set_rob_id(ownNumber);
    	for(i = 0; i < 8; i+=2)
    		e_set_led(i+1,1);
		sendRobotInfosBT(ownGroup,ownNumber);
		doSlaveStuffLoop();
    }
    // Group 2
    else if (selector == 4){
    	ownGroup = 2;
    	ownNumber = 1;

    	for(i = 0; i < 8; i+=2)
    		e_set_led(i,1);

		sendRobotInfosBT(ownGroup,ownNumber);
		while(1);
    }
    else if (selector == 5){
    	ownGroup = 2;
    	ownNumber = 2;
    	for(i = 0; i < 8; i+=2)
    		e_set_led(i,1);
		sendRobotInfosBT(ownGroup,ownNumber);
		while(1);
    }
    else if (selector == 6){
    	ownGroup = 2;
    	ownNumber = 3;
    	for(i = 0; i < 8; i+=2)
    		e_set_led(i,1);
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


// group is 1 or 2
// the leader will send group*10 
// and the slave group*10+slavenumber

void call_update_self_motion(void){

	//char tmp[255];
	sprintf(tmp, "Message robot: %d distance: %f direction: %f\n\r",(val-(val/10)*10), distance, direction);
			//btcomSendString(tmp);
	update_self_motion(msl,msr);
}


void sendPing(){
	ircomSend((int)ownGroup*10+ownNumber);	
	while (ircomSendDone() == 0); 
}

int isPartOfGroup(int message){
	int receivedGroup = (int) message/10;
	return ownGroup == receivedGroup;
}

void braitenAndComm(void){
	static int t = 0;
	// we wait that the message has been sent
	e_set_body_led(1);
	//we check and update the received messages
	IrcomMessage imsg;
	do{//while we have messages in the queue
	    ircomPopMessage(&imsg);
	    
	    //e_set_body_led(0);
	    if (imsg.error == 0){//we get a good message so we update the robot relative position
	    	int val = (int)imsg.value;
	    	double distance = (double)imsg.distance;
	    	double direction = (double)imsg.direction;
	    	updateRobotsPosition(val, distance, direction);

	    	//char tmp[255];
			//sprintf(tmp, "Message robot: %d distance: %f direction: %f\n\r",(val-(val/10)*10), distance, direction);
			//btcomSendString(tmp);
	    	//
	    }
	}while (imsg.error != -1);
	e_set_body_led(0);
	//we avoid any obstacle ()
	//braitenberg(bSpeeds);
	// we say all the other robots who we are
	if(t>2){
		e_set_body_led(1);
		sendPing();
		e_set_body_led(0);
		t = 0;
	}
	else
		t -=- 1;
}

void sendBtUpdate(){
	float x,y;
	get_self_position(&x,&y);
	char tmp[255];
	sprintf(tmp, "\n\r\n\rRobot N°: %d Group N° %d\n\r 1 Position X: %f\t 1 Position Y: %f\n\r2 Position X: %f\t 2 Position Y: %f\n\r3 Position X: %f\t 3 Position Y: %f\n\r",ownNumber,ownGroup, x, y, posX[1], posY[1], posX[2], posY[2]);
	btcomSendString(tmp);
}

void doLeaderStuffLoop(void){
	bool execute = true;

	//sendPing();
	e_activate_agenda(braitenAndComm, 1000); //every 500ms we do obstacle sensing/avoidance
	//e_activate_agenda(sendBtUpdate, 50000);
	e_activate_agenda(call_update_self_motion,DELTA_T*10000);/*(int)DELTA_T*10000);*/

	while (execute == true){
		reynolds_rules();

		compute_wheel_speeds(&msl,&msr);
		//get_wheel_speeds(&rSpeeds[LEFT], &rSpeeds[RIGHT]);

		//braitenberg(bSpeeds);
		//update wheel speeds

		msl += bSpeeds[LEFT];
		msr += bSpeeds[RIGHT]; 
		e_set_speed_left(msl);
		e_set_speed_right(msr);

		//wait(10);

	}

}

void doSlaveStuffLoop(void){
	while(1){};
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



void updateRobotsPosition(int val, double distance, double heading){
	if ((int)val/10 == ownGroup){
		int robotNumber = (int)(val-(val/10)*10);
		if(robotNumber > 0 && robotNumber < NB_ROBOTS_PER_GROUP){

			// the angle is rotated 90°
			posX[robotNumber-1] = cos(heading-M_PI/2.0)*distance;
			posY[robotNumber-1] = sin(heading-M_PI/2.0)*distance;

			if(heading>M_PI)
				heading = (-(2*M_PI-heading));


			process_received_ping_messages(robotNumber, distance/100, heading);
		}
	}
}
