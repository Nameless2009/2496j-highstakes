// motors
#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this

#include "main.h"

using namespace pros;

namespace glb

{
// defining ports
#define P_L_cata 10
#define P_R_cata 15
#define P_FR_chassis 1
#define P_FL_chassis 2
#define P_BR_chassis 3
#define P_BL_chassis 5
#define P_intake 4

    // objects

    Motor cataL(P_L_cata, E_MOTOR_GEARSET_18, true); // 18 is for speed (normal/green), 36 is for torque (red), 06 is for turbo (blue)
    Motor cataR(P_R_cata, E_MOTOR_GEARSET_18, false);
    Motor chassis_FR(P_FR_chassis,  E_MOTOR_GEARSET_18, true);
    Motor chassis_FL(P_FL_chassis, E_MOTOR_GEARSET_18, true);
    Motor chassis_BR(P_BR_chassis, E_MOTOR_GEARSET_18, true);
    Motor chassis_BL(P_BL_chassis, E_MOTOR_GEARSET_18, true);
    Motor intake(P_intake, E_MOTOR_GEARSET_18, true);

    Controller con(E_CONTROLLER_MASTER); // controller

}
#endif