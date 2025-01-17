#include "main.h"
#include "global.h"
#include "autons.h"

#include "api.h"
#include "bluecover.h"
#include "redcover.h"

using namespace glb;
using namespace pros;
using namespace std;


bool rush = false;
bool skills = false;
bool ring = false;
bool skip = false;

bool tipper = false;

bool doinkerState = false;

bool alliance = false;
int autoCycle = 0;

double DCSeconds = 0.00;
bool DPCleared = false;

int detectedTime;
bool timeLogged = false;
bool interrupt = false;
int interruptTime;

int driverProfileSequence = 0;

bool mogoState = false;

double releaseTime;

bool releaseRecorded;

int triggerTime;

double MreleaseTime;

bool MreleaseRecorded;

int MtriggerTime;

bool MmogoState = false;

bool ringLoadToggle = true;

bool redRing = false;

bool blueRing = false;

bool stallInterrupt = false;
bool stallLog = false;
double stallTime;
double reverseTime;

void stallProtection(){

	if (intake.get_actual_velocity() < 40.00 && stallLog == false){
		stallTime = pros::millis();
		stallLog = true;
	}
	if ((pros::millis() - stallTime) >= 500 && stallLog == true){
		stallInterrupt = true;
	}
	if (stallInterrupt == true){
		intake.move(-127);
		reverseTime = pros::millis();
		if ((pros::millis() - reverseTime) >= 500){
			intake.move(127);
			stallInterrupt = false;
			stallLog = false;
		}
	}


}

void senseColor(){
	colorSensor.set_led_pwm(100);
	while (1){
		if (colorSensor.get_proximity() <= 50){
			if ((colorSensor.get_hue()) >= 210.00 && colorSensor.get_hue() <= 250.00){
				blueRing = true;
			}
			else {
				blueRing = false;
			}
			if ((colorSensor.get_hue()) >= 330.00 && colorSensor.get_hue() <= 350.00){
				redRing = true;
			}
			else {
				redRing = false;
			}
		}
	}
}

void driverProfileAyush(){
	//tank control below
	double rightstick = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
	double leftstick = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

	rightChassis.move(rightstick);
	leftChassis.move(leftstick);

	//intake below
	if (con.get_digital(E_CONTROLLER_DIGITAL_R1) && interrupt == false){
		intake.move(127);
	}

	else if(con.get_digital(E_CONTROLLER_DIGITAL_R2) && interrupt == false){
		intake.move(-127);
	}

	else{
		intake.move(0);
	}

	//lady brown code below
	// ladyBrownCurrentPosition = (lbrotation.get_angle())/100;
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
		lbPID = true;
		// ringLoadToggle = !ringLoadToggle;
	}
	
	else{
		if (con.get_digital(E_CONTROLLER_DIGITAL_L1)){
			ladyBrown.move(127);
			lbPID = false;
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)){
			ladyBrown.move(-127);
			lbPID = false;
		}
		else{
			ladyBrown.move(0);
		}
	}

	// if (ringLoadToggle == false){
	// 	ladyBrownCorrectPosition = 342.00;
	// }
	// else if (ringLoadToggle == true){
	// 	ladyBrownCorrectPosition = 330.00;
	// }

	// double lberror = (ladyBrownCorrectPosition - ladyBrownCurrentPosition);

	// if (lbPID == true){
	// 	ladyBrown.move(ladyBrownPID(lberror, -3, -0, -0));
	// }

	//clamp and auto clamp code
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
		mogoState = !mogoState;
		if (mogoState == true){
			mogo.set_value(true);
		}
		else{
			mogo.set_value(false);
			releaseRecorded = false;
		}
	}
	if (mogoState == false && releaseRecorded == false){
		releaseTime = DCSeconds;
		releaseRecorded = true;
	}
	if (clampDistance.get() <= 200){
		triggerTime = DCSeconds;
	}
	if (clampDistance.get() <= 200 && (DCSeconds - triggerTime) >= 0.2 && (DCSeconds - releaseTime) > 2.00){
		mogoState = true;
		mogo.set_value(true);
	}

	//goal tipper code below
	if (con.get_digital(E_CONTROLLER_DIGITAL_X)){
		tipper = !tipper;
	}
	goalTipper.set_value(tipper);

	//doinker code below
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)){
		doinkerState = !doinkerState;
	}
	doinker.set_value(doinkerState);

	//color sort:
	if (blueRing == true && timeLogged == false){
		detectedTime = pros::millis();
		timeLogged = true;
	}

	if (timeLogged == true && (pros::millis() - detectedTime) >= 700){
		//interrupt = true; //UNCOMMENT THIS LATER
	}

	if (interrupt == true){
		intake.move(0);
		interruptTime = pros::millis();
		if ((pros::millis() - interruptTime) >= 500){
			interrupt = false;
			timeLogged = false;
			con.rumble(".");
		}
	}

}

void driverProfileManu(){
	//reg arcade control below
	double power = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
	double turn = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

	// if (curveX > 0){
	// 	//if positive just calculate normally
	// 	double curvedSpeed = (0.03*pow((curveX+40), 2))/(1+(16.430625/16129)*pow(curveX, 2))
	// }

	rightChassis.move(power - turn);
	leftChassis.move(power + turn);


	//intake below
	if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
		intake.move(127);
	}

	else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
		intake.move(-127);
	}

	else{
		intake.move(0);
	}

	//lady brown code below
	// ladyBrownCurrentPosition = (lbrotation.get_angle())/100;
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
		lbPID = true;
		// ringLoadToggle = !ringLoadToggle;
	}
	
	else{
		if (con.get_digital(E_CONTROLLER_DIGITAL_L1)){
			ladyBrown.move(127);
			lbPID = false;
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)){
			ladyBrown.move(-127);
			lbPID = false;
		}
		else{
			ladyBrown.move(0);
		}
	}

	// if (ringLoadToggle == false){
	// 	ladyBrownCorrectPosition = 342.00;
	// }
	// else if (ringLoadToggle == true){
	// 	ladyBrownCorrectPosition = 330.00;
	// }

	// double lberror = (ladyBrownCorrectPosition - ladyBrownCurrentPosition);

	// if (lbPID == true){
	// 	ladyBrown.move(ladyBrownPID(lberror, -3, -0, -0));
	// }

	//clamp and auto clamp code
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
		mogoState = !mogoState;
		if (mogoState == true){
			mogo.set_value(true);
		}
		else{
			mogo.set_value(false);
			releaseRecorded = false;
		}
	}
	// if (mogoState == false && releaseRecorded == false){
	// 	releaseTime = DCSeconds;
	// 	releaseRecorded = true;
	// }
	// if (clampDistance.get() <= 200){
	// 	triggerTime = DCSeconds;
	// }
	// if (clampDistance.get() <= 200 && (DCSeconds - triggerTime) >= 0.2 && (DCSeconds - releaseTime) > 2.00){
	// 	mogoState = true;
	// 	mogo.set_value(true);
	// }

	//goal tipper code below
	if (con.get_digital(E_CONTROLLER_DIGITAL_X)){
		tipper = !tipper;
	}
	goalTipper.set_value(tipper);

	//doinker code below
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)){
		doinkerState = !doinkerState;
	}
	doinker.set_value(doinkerState);
}

void driverProfileAnabel(){
	//reg arcade control below
	double power = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
	double turn = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

	rightChassis.move(power + turn);
	leftChassis.move(power - turn);


	//intake below
	if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
		intake.move(127);
	}

	else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
		intake.move(-127);
	}

	else{
		intake.move(0);
	}

	//mogo below
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
		mogoState = !mogoState;
	}

	mogo.set_value(mogoState);
}

void driverProfileLarry(){
	//inverted reverse arcade control below
	double power = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
	double turn = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

	if(power > 0){
		rightChassis.move(power + turn);
		leftChassis.move(power - turn);
	}
	else{
		rightChassis.move(power - turn);
		leftChassis.move(power + turn);
	}


	//intake below
	if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
		intake.move(127);
	}

	else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
		intake.move(-127);
	}

	else{
		intake.move(0);
	}

	//mogo below
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
		mogoState = !mogoState;
	}

	mogo.set_value(mogoState);
}

void driverProfileMessage(string messageSubject){
	if (messageSubject=="Ayush"){
		con.set_text(2,0, "DP:Ayush.");
	}
	else if (messageSubject=="Manu"){
		con.set_text(2,0, "DP:Manu..");
	}
	else if (messageSubject=="Larry"){
		con.set_text(2,0, "DP:Larry.");
	}
	else if (messageSubject=="Anabel"){
		con.set_text(2,0, "DP:Anabel");
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
	autoCycle--;
	if (autoCycle <= 0){
		autoCycle = 0;
		lcd::clear_line(3);
		lcd::set_text(3, "skipping auto");
		skip = true;
	}
	else if (autoCycle == 1){
		lcd::clear_line(3);
		lcd::set_text(3, "ring side selected");
		ring = true;
	}
	else if (autoCycle == 2){
		lcd::clear_line(3);
		lcd::set_text(3, "rush side selected");
		rush = true;
	}
	else if (autoCycle == 3){
		lcd::clear_line(3);
		lcd::set_text(3, "auto skills selected");
		skills = true;
	}
	else {
		autoCycle = 3;
	}
}
void on_left_button()
{
	alliance = !alliance;
	if (alliance == true){
		lcd::clear_line(2);
		lcd::set_text(2, "red alliance selected");
	}
	else{
		lcd::clear_line(2);
		lcd::set_text(2, "blue alliance selected");
	}
}
void on_right_button()
{
	autoCycle++;
	if (autoCycle <= 0){
		autoCycle = 0;
		lcd::clear_line(3);
		lcd::set_text(3, "skipping auto");
		skip = true;
	}
	else if (autoCycle == 1){
		lcd::clear_line(3);
		lcd::set_text(3, "ring side selected");
		ring = true;
	}
	else if (autoCycle == 2){
		lcd::clear_line(3);
		lcd::set_text(3, "rush side selected");
		rush = true;
	}
	else if (autoCycle == 3){
		lcd::clear_line(3);
		lcd::set_text(3, "auto skills selected");
		skills = true;
	}
	else {
		autoCycle = 3;
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
	pros::lcd::set_background_color(128, 0, 20);

	pros::lcd::set_text_color(194, 187, 169);
	pros::lcd::set_text(7, "...r/b...<----...---->...");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);

	//lvgl
	// lv_obj_t *red_cover = lv_img_create(lv_scr_act(), NULL);
	// lv_img_set_src(red_cover, &red_cover);
	// lv_obj_align(red_cover, NULL, LV_ALIGN_CENTER, 0, 0);


	ladyBrown.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	colorSensor.set_integration_time(3);

	pros::Task detectColors(senseColor);

	pros::Task lbTask(ladyBrownTask);

	pros::Task stallProtectionTask(stallProtection);
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
	// if (rush == true && alliance == true){
	// 	redRushSide();
	// }
	// else if (rush == true && alliance == false){
	// 	blueRushSide();
	// }
	// else if (ring == true && alliance == true){
	// 	redRingSide();
	// }
	// else if (ring == true && alliance == false){
	// 	blueRingSide();
	// }
	// else if (skills){
	// 	skillsAuto();
	// }
	// else { //if nothing was selected
	// 	skipAutonomous();
	// }
	skipAutonomous();
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
	lcd::clear();
	con.clear();
	con.clear_line(2);

	while (true)
	{
		if (DCSeconds <= 10.00){ //locks the change and print of driver profiles after 10 seconds
			if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
				driverProfileSequence++;
			}
			if (driverProfileSequence == 1){
				driverProfileMessage("Ayush");
			}
			else if(driverProfileSequence == 0){
				driverProfileMessage("Manu");
			}
			else if(driverProfileSequence == 2){
				driverProfileMessage("Larry");
			}
			else if(driverProfileSequence == 3){
				driverProfileMessage("Anabel");
			}
		}

		//which profile to run based on the sequencer value
		if (driverProfileSequence == 1){
			driverProfileAyush();
		}
		else if(driverProfileSequence == 0){
			driverProfileManu();
		}
		else if(driverProfileSequence == 2){
			driverProfileLarry();
		}
		else if(driverProfileSequence == 3){
			driverProfileAnabel();
		}
		else{
			driverProfileSequence = 0;
		}

		if (DCSeconds >= 10.00 && DPCleared == false){
			con.clear_line(2);
			DPCleared = true;
		}


		delay(10);
		DCSeconds += 0.01;
	}
}
