#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"

using namespace std;
using namespace pros;

void drivePID(int desiredValue, int timeout=1500);

void turnPID(int desiredValue, int timeout=1500);

float calculatePID(float error);

float calculatePID2(float error);

void leftArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0);

void rightArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0);

void program1();

void program2();

void program3();

void skipAutonomous();



#endif