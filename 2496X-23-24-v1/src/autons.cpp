#include "main.h"
#include "global.h"

using namespace pros;
using namespace glb;

void drivePID(int desiredValue){
	bool enableDrivePID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 0.687;
	double kI = 0.0023;
	double kD = 4.514;

	chassis.tare_position();

	while (enableDrivePID){
		//get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		//get avg of motors:
		int currentValue = (FRpos+BRpos+FLpos+BLpos)/4;

		//proportional
		int error = desiredValue - currentValue;

		//derivative
		int derivative = error - prevError;

		//integral
		if (abs(error) < 20){
			totalError += error;
		}


		double speed = (error*kP + derivative*kD + totalError*kI);
		chassis.move(speed);

		prevError = error;

		if (error < 5){
			count++;
		}

		if (count > 20){
			enableDrivePID = false;
		} 

		delay(20);
	}

	chassis.move(0);

}


void turnRightPID(int desiredValue){
	bool enableTurnPID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 0.687;
	double kI = 0.0023;
	double kD = 4.514;

	chassis.tare_position();

	while (enableTurnPID){
		//get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		//get avg of motors:
		int currentValue = (abs(FRpos)+abs(BRpos)+abs(FLpos)+abs(BLpos))/4;

		//proportional
		int error = desiredValue - currentValue;

		//derivative
		int derivative = error - prevError;

		//integral
		if (abs(error) < 20){
			totalError += error;
		}


		double speed = (error*kP + derivative*kD + totalError*kI);
		rightChassis.move(speed);
		leftChassis.move(-speed);

		prevError = error;

		if (error < 5){
			count++;
		}

		if (count > 20){
			enableTurnPID = false;
		} 

		delay(20);
	}

	chassis.move(0);

}


void turnLeftPID(int desiredValue){
	bool enableTurnPID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 0.687;
	double kI = 0.0023;
	double kD = 4.514;

	chassis.tare_position();

	while (enableTurnPID){
		//get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		//get avg of motors:
		int currentValue = (abs(FRpos)+abs(BRpos)+abs(FLpos)+abs(BLpos))/4;

		//proportional
		int error = desiredValue - currentValue;

		//derivative
		int derivative = error - prevError;

		//integral
		if (abs(error) < 20){
			totalError += error;
		}


		double speed = (error*kP + derivative*kD + totalError*kI);
		rightChassis.move(-speed);
		leftChassis.move(speed);

		prevError = error;

		if (error < 5){
			count++;
		}

		if (count > 20){
			enableTurnPID = false;
		} 

		delay(20);
	}

	chassis.move(0);

}






//pseudocode
void offSide(){
	turnRightPID(1000); //turn around 180 deg
	drivePID(200);
	turnLeftPID(250); //allign with matchloading bar
	wings.set_value(true); //extend wings
	drivePID(500); //remove triball from match load area
	turnLeftPID(250); //turn to be straight
	drivePID(1000); //push triballs across border
}

void onSide(){
	// MAKE SURE TO START AUTON WITH HALF CATA
	drivePID(1500); //drive forward toward goal
	turnRightPID(250); //turn a little to allign with goal
	intake.move_relative(-300, 127); //reverse intake to spit out triball
	drivePID(250); //push alliance triball into goal
	drivePID(-250); //reverse
	turnLeftPID(425); // turn to align with first triball
	intake.move(127); //spin intake forward
	drivePID(2000); //drive toward first triball to intake it
	turnRightPID(900); //face goal
	intake.move(-127); //outake towards goal
	turnLeftPID(500); //face next triball
	intake.move(127); //intake spin forward
	drivePID(500); //drive to intake it
	turnRightPID(500); //face goal
	intake.move(-127); //outake towards goal
	wings.set_value(true); //extend wings
	drivePID(1500); //push everything in the goal
	wings.set_value(false); //retract wings 
	drivePID(-150); //reverse
	turnRightPID(500);
	drivePID(1500);
	turnRightPID(500);
	wings.set_value(true);
	drivePID(1000); //touch bar for AWP
}

void skipAuton(){

}
