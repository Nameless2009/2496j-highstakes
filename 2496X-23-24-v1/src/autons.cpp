#include "main.h"
#include "global.h"

using namespace pros;
using namespace glb;

void driveForwardPID(int desiredValue)
{
	bool enableDrivePID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 0.67;
	double kI = 0.00273; 
	double kD = 2.7;

	chassis.tare_position();

	while (enableDrivePID)
	{
		// get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		// get avg of motors:
		int currentValue = (FRpos + BRpos + FLpos + BLpos) / 4;

		// proportional
		int error = desiredValue - currentValue;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < 20)
		{
			totalError += error;
		}

		double speed = (error * kP + derivative * kD + totalError * kI);
		chassis.move(speed);

		prevError = error;

		if (error < 10)
		{
			count++;
		}

		if (count > 5)
		{
			enableDrivePID = false;
		}

		delay(20);
	}

	chassis.move(0);
}

void driveBackwardPID(int desiredValue)
{
	bool enableDrivePID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 0.67;
	double kI = 0.00273; 
	double kD = 2.7;

	chassis.tare_position();

	while (enableDrivePID)
	{
		// get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		// get avg of motors:
		int currentValue = (abs(FRpos) + abs(BRpos) + abs(FLpos) + abs(BLpos)) / 4;

		// proportional
		int error = desiredValue - currentValue;


		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < 20)
		{
			totalError += error;
		}

		double speed = (error * kP + derivative * kD + totalError * kI);
		chassis.move(-speed);

		prevError = error;

		if (error < 10)
		{
			count++;
		}

		if (count > 5)
		{
			enableDrivePID = false;
		}

		delay(20);
	}

	chassis.move(0);
}

void turnRightPID(int desiredValue)
{
	bool enableTurnPID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 2.5;
	double kI = 0.0009645;
	double kD = 7.49535;

	chassis.tare_position();

	while (enableTurnPID)
	{
		// get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		// get avg of motors:
		int currentValue = (abs(FRpos) + abs(BRpos) + abs(FLpos) + abs(BLpos)) / 4;

		// proportional
		int error = desiredValue - currentValue;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < 20)
		{
			totalError += error;
		}

		double speed = (error * kP + derivative * kD + totalError * kI);
		rightChassis.move(speed);
		leftChassis.move(-speed);

		prevError = error;

		if (error < 10)
		{
			count++;
		}

		if (count > 5)
		{
			enableTurnPID = false;
		}

		delay(20);
	}

	chassis.move(0);
}

void turnLeftPID(int desiredValue)
{
	bool enableTurnPID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 2.5;
	double kI = 0.0009645;
	double kD = 7.49535;

	chassis.tare_position();

	while (enableTurnPID)
	{
		// get position of all motors:
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

		// get avg of motors:
		int currentValue = (abs(FRpos) + abs(BRpos) + abs(FLpos) + abs(BLpos)) / 4;

		// proportional
		int error = desiredValue - currentValue;


		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < 20)
		{
			totalError += error;
		}

		double speed = (error * kP + derivative * kD + totalError * kI);
		rightChassis.move(-speed);
		leftChassis.move(speed);

		prevError = error;

		if (error < 10)
		{
			count++;
		}

		if (count > 5)
		{
			enableTurnPID = false;
		}

		delay(20);
	}

	chassis.move(0);
}


void offSide(){ //make so all pneumatics start closed in init
	zoneMech.set_value(true); //switch to zone mech later
	delay(300);
	turnLeftPID(180);
	delay(50);
	turnRightPID(300);
	zoneMech.set_value(false);
	delay(20);
	intake.move_relative(-500, 127);
	// driveForwardPID(1000);
	chassis.move(127);
	delay(850);
	// chassis.move_relative(-1000, 127); //this goes forward for some reason
	chassis.move(0);
	driveBackwardPID(650);
	delay(10);
	turnRightPID(500);
	chassis.move(-100);
	delay(500);
	chassis.move(0);
	driveForwardPID(500);
	turnRightPID(250);
	driveForwardPID(1150);
	turnRightPID(600);
	driveBackwardPID(1800);
}

void onSide(){

	intake.move(127);
	delay(500);
	driveForwardPID(2100);
	turnRightPID(500);
	intake.move(-127);
	delay(800);
	turnLeftPID(500);
	intake.move(127);
	driveForwardPID(1000);
	turnRightPID(600);
	intake.move(-127);
	wings.set_value(true);
	chassis.move(127);
	delay(1000);
	driveBackwardPID(1000);
}

void autonSkills(){
	// cata.move(100);
	// delay(50000); //wait 50 seconds
	// cata.move(0);
	// driveBackwardPID(500);
	// turnLeftPID(300);
	// driveForwardPID(1500);
	// turnLeftPID(150);
	// while(catalimit.get_value()==false){ //until limitswitch pressed spin cata. when limit pressed tare pos
	// 	cata.move(127);
	// }
	// cata.tare_position();
	// cata.move_relative(100, 127); //move cata to top
	// chassis.move(70); //move chassis slowly toward bar to hang
}

void skipAutonomous(){
	
}