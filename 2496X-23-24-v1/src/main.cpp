#include "main.h"
#include "global.h"
#include "autons.h"

using namespace glb;
using namespace pros;
using namespace std;

//fix?

//this is the current version, create new branch and push.
bool startCata = false;
bool stopCata = false;
bool antiJamOverride = false;
bool matchLoadingMode = false;
bool wingValue = false;
bool intakeLifterValue = true;


//delete later (after testing)
// void drivePID(int desiredValue){
// 	bool enableDrivePID = true;
// 	int prevError = 0;
// 	int totalError = 0;
// 	int count = 0;

// 	double kP = 0.687;
// 	double kI = 0.0023;
// 	double kD = 4.514;

// 	chassis.tare_position();

// 	while (enableDrivePID){
// 		//get position of all motors:
// 		int FRpos = chassis_FR.get_position();
// 		int FLpos = chassis_FL.get_position();
// 		int BRpos = chassis_BR.get_position();
// 		int BLpos = chassis_BL.get_position();

// 		//get avg of motors:
// 		int currentValue = (FRpos+BRpos+FLpos+BLpos)/4;

// 		//proportional
// 		int error = desiredValue - currentValue;

// 		//derivative
// 		int derivative = error - prevError;

// 		//integral
// 		if (abs(error) < 20){
// 			totalError += error;
// 		}


// 		double speed = (error*kP + derivative*kD + totalError*kI);
// 		chassis.move(speed);

// 		prevError = error;

// 		if (error < 5){
// 			count++;
// 		}

// 		if (count > 20){
// 			enableDrivePID = false;
// 		} 

// 		delay(20);
// 	}

// 	chassis.move(0);

// }


// void turnRightPID(int desiredValue){
// 	bool enableTurnPID = true;
// 	int prevError = 0;
// 	int totalError = 0;
// 	int count = 0;

// 	double kP = 0.687;
// 	double kI = 0.0023;
// 	double kD = 4.514;

// 	chassis.tare_position();

// 	while (enableTurnPID){
// 		//get position of all motors:
// 		int FRpos = chassis_FR.get_position();
// 		int FLpos = chassis_FL.get_position();
// 		int BRpos = chassis_BR.get_position();
// 		int BLpos = chassis_BL.get_position();

// 		//get avg of motors:
// 		int currentValue = (abs(FRpos)+abs(BRpos)+abs(FLpos)+abs(BLpos))/4;

// 		//proportional
// 		int error = desiredValue - currentValue;

// 		//derivative
// 		int derivative = error - prevError;

// 		//integral
// 		if (abs(error) < 20){
// 			totalError += error;
// 		}


// 		double speed = (error*kP + derivative*kD + totalError*kI);
// 		rightChassis.move(speed);
// 		leftChassis.move(-speed);

// 		prevError = error;

// 		if (error < 5){
// 			count++;
// 		}

// 		if (count > 20){
// 			enableTurnPID = false;
// 		} 

// 		delay(20);
// 	}

// 	chassis.move(0);

// }


// void turnLeftPID(int desiredValue){
// 	bool enableTurnPID = true;
// 	int prevError = 0;
// 	int totalError = 0;
// 	int count = 0;

// 	double kP = 0.687;
// 	double kI = 0.0023;
// 	double kD = 4.514;

// 	chassis.tare_position();

// 	while (enableTurnPID){
// 		//get position of all motors:
// 		int FRpos = chassis_FR.get_position();
// 		int FLpos = chassis_FL.get_position();
// 		int BRpos = chassis_BR.get_position();
// 		int BLpos = chassis_BL.get_position();

// 		//get avg of motors:
// 		int currentValue = (abs(FRpos)+abs(BRpos)+abs(FLpos)+abs(BLpos))/4;

// 		//proportional
// 		int error = desiredValue - currentValue;

// 		//derivative
// 		int derivative = error - prevError;

// 		//integral
// 		if (abs(error) < 20){
// 			totalError += error;
// 		}


// 		double speed = (error*kP + derivative*kD + totalError*kI);
// 		rightChassis.move(-speed);
// 		leftChassis.move(speed);

// 		prevError = error;

// 		if (error < 5){
// 			count++;
// 		}

// 		if (count > 20){
// 			enableTurnPID = false;
// 		} 

// 		delay(20);
// 	}

// 	chassis.move(0);

// }


void cataCycle(){
	if (matchLoadingMode == false){
		if (startCata == true){
			cata.move(127);
		}
		else{
			cata.move(0);
		}

		if (stopCata == true){
			cata.move(0);
			stopCata = false;
		}
	}
	else if (matchLoadingMode == true){
		cata.move(90);
	}
}

void refresh(){
    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
        startCata = !startCata;
    }
    if (catalimit.get_new_press()){
        stopCata = true;
        startCata = false;
    }
}

void cataCode(){
    refresh();
    cataCycle();
}

void functionChangeButtonCode(){
	if (con.get_digital(E_CONTROLLER_DIGITAL_L2)){
		if (con.get_digital(E_CONTROLLER_DIGITAL_L1)){
			blocker.move(80);
		}
		else{
			blocker.move(0);
		}
		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
			wingValue = !wingValue;
			wings.set_value(wingValue);
		}
		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
			intakeLifterValue = !intakeLifterValue;
			intakeLifter.set_value(intakeLifterValue);
		}
	}
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		lcd::set_text(2, "I was pressed!");
	}
	else
	{
		lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "2496");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

	while (true)
	{
		double rightstick = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		double leftstick = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

		chassis_FR.move(rightstick);
		chassis_BR.move(rightstick);
		chassis_FL.move(leftstick);
		chassis_BL.move(leftstick);


        //cata code:
        cataCode();

		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
			matchLoadingMode = !matchLoadingMode;
		}

		functionChangeButtonCode();

		if (con.get_digital(E_CONTROLLER_DIGITAL_X)){ //pressing
			blocker.move(-80); //retract
		}


		if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
			intake.move(127);
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)){
			intake.move(-127);
		}
		else {
			intake.move(0);
		}
		delay(2);
	}
}
