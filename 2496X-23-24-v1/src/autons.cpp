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

	con.clear();

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

		con.print(0,0, "error: %d", error);

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

		con.clear();
		con.print(0,0, "error: %d", error);

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
	con.clear();

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

		con.print(0,0, "error: %d", error);

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

		con.clear();
		con.print(0,0, "error: %d", error);

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


void offSide(){
	driveBackwardPID(1000);
	turnRightPID(250);
	driveBackwardPID(500);
	driveForwardPID(500);
	turnLeftPID(250);
	zoneMech.set_value(true);
	driveForwardPID(1000);
	turnLeftPID(250);
	zoneMech.set_value(false);
	driveForwardPID(3000);
	blocker.move_relative(500, 127);
}

void onSide(){
	intake.move(127);
	driveForwardPID(500);
	turnRightPID(1000);
	intake.move(0);
	driveForwardPID(2000);
	turnLeftPID(250);
	driveForwardPID(1000);
	turnLeftPID(250);
	driveForwardPID(800);
	driveBackwardPID(800);
	turnLeftPID(450);
	intake.move(127);
	driveForwardPID(2500);
	turnRightPID(450);
	intake.move(-127);
	turnLeftPID(450);
	intake.move(127);
	driveForwardPID(500);
	turnRightPID(500);
	intake.move(-127);
	wings.set_value(true);
	driveForwardPID(2500);
}

void autonSkills(){
	cata.move(100);
	delay(50000); //wait 50 seconds
	cata.move(0);
	driveBackwardPID(500);
	turnLeftPID(300);
	driveForwardPID(1500);
	turnLeftPID(150);
	while(catalimit.get_value()==false){ //until limitswitch pressed spin cata. when limit pressed tare pos
		cata.move(127);
	}
	cata.tare_position();
	cata.move_relative(100, 127); //move cata to top
	chassis.move(70); //move chassis slowly toward bar to hang
}

void skipAutonomous(){
	
}