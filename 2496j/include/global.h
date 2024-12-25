#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this


#include "main.h"


using namespace pros;


namespace glb
{
    // defining ports
    #define P_FL_chassis 18
    #define P_FR_chassis 2
    #define P_ML_chassis 11
    #define P_MR_chassis 1
    #define P_BL_chassis 3
    #define P_BR_chassis 10

    #define P_intake 20

    #define P_ladyBrown 12

    #define P_mogo 'A'

    #define P_inertial 19

    #define P_rotation 16

    #define P_distance 10

    #define P_color_sensor 13

    #define P_doinker 'C'


    // objects
    extern Motor FR;
    extern Motor FL;
    extern Motor BR;
    extern Motor BL;
    extern Motor RM;
    extern Motor LM;

    extern Motor intake;

    extern Motor_Group rightChassis;
    extern Motor_Group leftChassis;
    extern Motor_Group chassis;

    extern Motor ladyBrown;

    extern Imu inertial;

    extern ADIDigitalOut mogo;

    extern ADIDigitalOut doinker;

    extern Controller con;

    extern Rotation lbrotation;
    
    extern Distance clampDistance;

    extern Optical colorSensor;

}
#endif
