#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"


using namespace pros;

void driveForwardPID(int desiredValue);

void driveBackwardPID(int desiredValue);

void turnPID(int desiredValue);

void offSide();

void onSide();

void autonSkills();

void skipAutonomous();



#endif