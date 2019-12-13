
#ifndef _MAIN
#define _MAIN

void obstacleAvoidance();

void leaderPingSlave();
void slaveRespondsLeader();
int isPartOfGroup(int message);
void doLeaderStuffLoop(void);
void doSlaveStuffLoop(void);
void sendRobotInfosBT(int group, int number);
void updateRobotsPosition(int val, double distance, double heading);

void braitenAndComm(void);


#endif

