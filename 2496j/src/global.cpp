#include "global.h"
#include "autons.h"


namespace glb {
    // objects

    Motor FR(P_FR_chassis, E_MOTOR_GEARSET_18, false);
    Motor FL(P_FL_chassis, E_MOTOR_GEARSET_18, true);
    Motor BR(P_BR_chassis, E_MOTOR_GEARSET_18, false);
    Motor BL(P_BL_chassis, E_MOTOR_GEARSET_18, true);
    Motor RM(P_MR_chassis, E_MOTOR_GEAR_600, false);
    Motor LM(P_ML_chassis, E_MOTOR_GEAR_600, true);


    Motor_Group rightChassis({P_FR_chassis, P_BR_chassis, P_MR_chassis});
    Motor_Group leftChassis({P_FL_chassis, P_BL_chassis, P_ML_chassis});
    Motor_Group chassis({P_FR_chassis, P_BR_chassis, P_FL_chassis, P_BL_chassis, P_ML_chassis, P_MR_chassis});

    Motor intake(P_intake, E_MOTOR_GEAR_600, true);

    Motor ladyBrown(P_ladyBrown, E_MOTOR_GEAR_200, false);

    ADIDigitalOut mogo(P_mogo, false);

    ADIDigitalOut doinker(P_doinker, false);

    ADIDigitalOut intakeLift(P_intakeLift, false);

    ADIDigitalOut goalTipper(P_goalTipper, false);

    Imu inertial(P_inertial);

    Controller con(E_CONTROLLER_MASTER); // controller

    Rotation lbrotation(P_rotation);

    Distance clampDistance(P_distance);

    Optical colorSensor(P_color_sensor);
}

using namespace glb;

bool lbPID = false;
double ladyBrownCorrectPosition = 329.00;
double ladyBrownCurrentPosition;

void ladyBrownTask(){
	while(1){
		ladyBrownCurrentPosition = (lbrotation.get_angle())/100;
		double lberror = (ladyBrownCorrectPosition - ladyBrownCurrentPosition);
		if (lbPID == true){
			ladyBrown.move(ladyBrownPID(lberror, -3, -0, -0));
		}
	}
}
