#include "main.h"
#include "global.h"

using namespace glb;
using namespace pros;
using namespace std;

//organize code when you have time

//this is the current version, create new branch and push.
bool startCata = false;
bool stopCata = false;
bool antiJamOverride = false;
bool matchLoadingMode = false;

void cataCycle(){
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

void refresh(){
    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
        startCata = !startCata;
    }
    if (catalimit.get_new_press()){
        stopCata = true;
        startCata = false;
    }
    // if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
    //     antiJamOverride = !antiJamOverride;
    //     con.rumble(".");
    // }
}

void antiJam(){
    if (antiJamOverride == false){
        if (catalimit.get_value()==false){
            intake.move(0);
            //stop intake if limit not pressed (preventing jamming)
        }
    }
}

void cataCode(){
    refresh();
    // antiJam();
    cataCycle();
}

void matchLoadingCode(){
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
		matchLoadingMode = !matchLoadingMode;
	}
	if (matchLoadingMode==true){
		cata.move(127);
	}
	else{
		cata.move(0);
	}
}


// void limit(){
//     if (startCata == false || stopCata == true){
//         cata.move(0);
//         stopCata = false;
//     }
//     else{
//         cata.move(100);
//     }
// }

// void half(){
//     if (x==0){
//         limit(); //run regular limit switch code
//         cata.tare_position();
//         x = 1;
//     }
//     else if (x==1){
//         //move to half position:
//         if (cataL.get_position()>300 && cataL.get_position()<350){
//             cata.move(0);
//             x=0;
//         }
//         else {
//             cata.move(100);
//         }
//     }
// }


// void cataCode(){
//     if (mode == false){
//         limit();
//     }
//     else if (mode == true){
//         half();
//     }
// }

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
void autonomous() {}

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

		matchLoadingCode();

        

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
