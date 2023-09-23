// motors
#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this

#include "main.h"

using namespace pros;

namespace glb
{
// defining ports
#define P_L_CATA 10
#define P_R_CATA 15

    // objects
    Motor cataL(P_L_CATA, E_MOTOR_GEARSET_18, true); // 18 is for speed, 36 is for turbo
    Motor cataR(P_R_CATA, E_MOTOR_GEARSET_18, false);
    extern Motor cataL;

    Controller con(E_CONTROLLER_MASTER); // controller

}
#endif