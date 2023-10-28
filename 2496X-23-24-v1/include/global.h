#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this


#include "main.h"


using namespace pros;


namespace glb


{
// defining ports
#define P_L_cata 8
#define P_R_cata 15
#define P_FL_chassis 1
#define P_BR_chassis 2
#define P_BL_chassis 3
#define P_FR_chassis 5
#define P_intake 9
#define P_catalimit 8 //switch to 8 later
#define P_imu 21
#define P_intakeLifter 1
#define P_wings 7
#define P_blocker 4


    // objects


    extern Motor cataL;
    extern Motor cataR;
    extern Motor chassis_FR;
    extern Motor chassis_FL;
    extern Motor chassis_BR;
    extern Motor chassis_BL;
    extern Motor intake;
    extern Motor_Group rightChassis;
    extern Motor_Group leftChassis;
    extern Motor_Group chassis;

    extern Motor blocker;

    extern ADIDigitalOut intakeLifter;
    extern ADIDigitalOut wings;


    extern Motor_Group cata;


    // Imu inertial(P_imu);


    extern ADIDigitalIn catalimit;


    extern Controller con;


}
#endif
