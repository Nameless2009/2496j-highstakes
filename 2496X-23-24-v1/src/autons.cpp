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

	double kP = 1.46;
	double kI = 0.0782;
	double kD = 6.41;
	// need to retune ^

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

		if (error < 5)
		{
			count++;
		}

		if (count > 0)
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

	double kP = 1.46;
	double kI = 0.0782;
	double kD = 6.41;
	// need to retune ^

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

		if (error < 5)
		{
			count++;
		}

		if (count > 0)
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

	double kP = 1.46;
	double kI = 0.0782;
	double kD = 6.41;

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

		if (error < 5)
		{
			count++;
		}

		if (count > 0)
		{
			enableTurnPID = false;
		}

		delay(20);

		lcd::clear_line(1);
		lcd::print(1, "error: %d", error);
	}

	chassis.move(0);
}

void turnLeftPID(int desiredValue)
{
	bool enableTurnPID = true;
	int prevError = 0;
	int totalError = 0;
	int count = 0;

	double kP = 1.46;
	double kI = 0.0782;
	double kD = 6.41;

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

		if (error < 5)
		{
			count++;
		}

		if (count > 0)
		{
			enableTurnPID = false;
		}

		delay(20);
	}

	chassis.move(0);
}


void offSide(){

}

void onSide(){

}

void skipAuton(){

}