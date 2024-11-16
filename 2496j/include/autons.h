#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"

using namespace std;
using namespace pros;

void driveSPID(int desiredValue, int timeout=1500, int chainSpeed=0);

void drivePID(int desiredValue, int timeout=1500, int chainSpeed=0, bool autoclamp=false);

void drivePIDMogo(int desiredValue, int timeout=1500, int chainSpeed=0);

void turnPID(int desiredValue, int timeout=1500, bool powerFunc = true);

void turnPIDMogo(int desiredValue, int timeout=1500, bool powerFunc=true);

double ladyBrownPID(double error, double kP=-2, double kI=-0.1, double kD=-0.8, double totalError=0, double prevError=0, double integralThreshold=30, double maxI=500);

float calculatePID(float error);

float calculatePID2(float error);

void leftArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0);

void rightArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0);

void redRingSide();

void blueRingSide();

void redRushSide();

void blueRushSide();

void skillsAuto();

void skipAutonomous();



#endif
