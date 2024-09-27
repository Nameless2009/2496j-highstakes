#include "main.h"
#include "global.h"
#include "autons.h"

using namespace glb;
using namespace pros;
using namespace std;



// bool startCata = false;
// bool stopCata = false;
// bool antiJamOverride = false;
// bool matchLoadingMode = false;
// bool wingsToggle = false;
// bool intakeLifterValue = true;
// bool half = false;

bool auto1 = false;
bool auto2 = false;
bool auto3 = false;



// void cataCycle(){
// 	if (matchLoadingMode == false){
// 		if (startCata == true){
// 			cata.move(127);
// 		}
// 		else{
// 			cata.move(0);
// 		}

// 		if (stopCata == true){
// 			cata.move(0);
// 			stopCata = false;
// 		}
// 	}
// 	else if (matchLoadingMode == true){
// 		cata.move(120);
// 	}
// }

// void refresh(){
//     if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
//         startCata = !startCata;
//     }
//     if (catalimit.get_new_press()){
//         stopCata = true;
//         startCata = false;
//     }
// }

// void cataCode(){
//     refresh();
//     cataCycle();
// }


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	auto1 = true;
	if (auto1)
	{
		lcd::clear();
		lcd::set_text(3, "(AUTO1) selected.");
	}
	else
	{
		lcd::clear_line(3);
	}
}
void on_left_button()
{
	auto2 = true;
	if (auto2)
	{
		lcd::clear();
		lcd::set_text(3, "(AUTO2) selected.");
	}
	else
	{
		lcd::clear_line(3);
	}
}
void on_right_button()
{
	auto3 = true;
	if (auto3)
	{
		lcd::clear();
		lcd::set_text(3, "(AUTO3) selected.");
	}
	else
	{
		lcd::clear_line(3);
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

	// blocker.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	// wings.set_value(false);

	
	pros::lcd::set_text(1, "Left Button: (AUTO1)");
	pros::lcd::set_text(2, "Center Button: (AUTO2)");
	pros::lcd::set_text(3, "Right Button: (AUTO3)");
	pros::lcd::set_text(4, "Click Nothing: SKIP AUTON");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
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
	if (auto1){
		program1();
	}
	else if (auto2){
		program2();
	}
	else if (auto3){
		program3();
	}
	else { //if nothing was clicked
		skipAutonomous();      
	}
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
	// wings.set_value(true);
	lcd::clear();

	while (true)
	{
		double rightstick = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		double leftstick = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

		rightChassis.move(rightstick);
		leftChassis.move(leftstick);

		if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
			lower_intake.move(127);
			top_intake.move(127);
		}

		else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
			lower_intake.move(-127);
			top_intake.move(-127);
		}

		else{
			lower_intake.move(0);
			top_intake.move(0);
		}


        // cataCode();

		// if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
		// 	matchLoadingMode = !matchLoadingMode;
		// }

		// if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
		// 	wingsToggle = !wingsToggle;
		// }
		// if (wingsToggle==false){
		// 	wings.set_value(false);
		// }else {
		// 	wings.set_value(true);
		// }

		// if (con.get_digital(E_CONTROLLER_DIGITAL_LEFT)){
		// 	blockerLeft.move(127);
		// 	blockerRight.move(127);
		// }
		// else if(con.get_digital(E_CONTROLLER_DIGITAL_RIGHT)){
		// 	blockerLeft.move(-127);
		// 	blockerRight.move(-127);
		// }
		// else{
		// 	blockerLeft.move(0);
		// 	blockerRight.move(0);
		// }


		// if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
		// 	intake.move(127);
		// }
		// else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)){
		// 	intake.move(-127);
		// }
		// else {
		// 	intake.move(0);
		// }

		delay(2);

	}
}