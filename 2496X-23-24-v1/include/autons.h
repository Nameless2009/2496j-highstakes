#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"


using namespace pros;

void drivePID(int desiredValue);

void turnRightPID(int desiredValue);

void turnLeftPID(int desiredValue);

void offSide();

void onSide();

void skipAuton();



#endif
