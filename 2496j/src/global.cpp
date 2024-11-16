#include "global.h"


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

    Motor ladyBrownRight(P_ladyBrownRight, E_MOTOR_GEAR_200, false);
    Motor ladyBrownLeft(P_ladyBrownLeft, E_MOTOR_GEAR_200, true);

    Motor_Group ladyBrown({P_ladyBrownLeft, P_ladyBrownRight});

    ADIDigitalOut mogo('A', false);

    ADIDigitalOut doinker('C', false);

    Imu inertial(P_inertial);

    Controller con(E_CONTROLLER_MASTER); // controller

    Rotation lbrotation(P_rotation);

    Distance clampDistance(P_distance);
}
