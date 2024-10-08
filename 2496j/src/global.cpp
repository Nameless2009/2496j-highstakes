#include "global.h"


namespace glb {
    // objects

    Motor FR(P_FR_chassis, E_MOTOR_GEARSET_18, false);
    Motor FL(P_FL_chassis, E_MOTOR_GEARSET_18, true);
    Motor BR(P_BR_chassis, E_MOTOR_GEARSET_18, false);
    Motor BL(P_BL_chassis, E_MOTOR_GEARSET_18, true);
    Motor RM(P_RM_chassis, E_MOTOR_GEAR_600, false);
    Motor LM(P_LM_chassis, E_MOTOR_GEAR_600, true);


    Motor_Group rightChassis({P_FR_chassis, P_BR_chassis, P_RM_chassis});
    Motor_Group leftChassis({P_FL_chassis, P_BL_chassis, P_LM_chassis});
    Motor_Group chassis({P_FR_chassis, P_BR_chassis, P_FL_chassis, P_BL_chassis, P_RM_chassis, P_LM_chassis}); 

    Motor top_intake(P_top_intake, E_MOTOR_GEAR_600, true);
    Motor lower_intake(P_lower_intake, E_MOTOR_GEAR_600, true);

    Motor lift(P_lift, E_MOTOR_GEAR_200, false);

    ADIDigitalOut mogo('A', false);

    Imu inertial(P_inertial);

    Controller con(E_CONTROLLER_MASTER); // controller


}
