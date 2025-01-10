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

    #define P_intake 4

    #define P_ladyBrown 12

    #define P_mogo 'B'

    #define P_intakeLift 'A'

    #define P_goalTipper 'D'

    #define P_inertial 21

    #define P_rotation 16

    #define P_distance 14

    #define P_color_sensor 13

    #define P_doinker 'H'


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

    extern ADIDigitalOut goalTipper;

    extern ADIDigitalOut intakeLift;

    extern Controller con;

    extern Rotation lbrotation;
    
    extern Distance clampDistance;

    extern Optical colorSensor;

}

extern bool lbPID;
extern double ladyBrownCorrectPosition;
extern double ladyBrownCurrentPosition;

extern void ladyBrownTask();

#endif
