#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this


#include "main.h"


using namespace pros;


namespace glb


{
    // defining ports
    #define P_FR_chassis 9 
    #define P_BL_chassis 1
    #define P_BR_chassis 11
    #define P_FL_chassis 7
    #define P_LM_chassis 4
    #define P_RM_chassis 12

    #define P_top_intake 10
    #define P_lower_intake 9

    #define P_lift 21

    #define P_inertial 20


    // objects
    extern Motor FR;
    extern Motor FL;
    extern Motor BR;
    extern Motor BL;
    extern Motor RM;
    extern Motor LM;

    extern Motor top_intake;
    extern Motor lower_intake;

    extern Motor_Group rightChassis;
    extern Motor_Group leftChassis;
    extern Motor_Group chassis;

    extern Motor lift;

    extern Imu inertial;

    extern ADIDigitalOut mogo;

    extern Controller con;


}
#endif
