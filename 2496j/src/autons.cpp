#include "main.h"
#include "global.h"

using namespace pros;
using namespace glb;
using namespace std;


// void drivePID(int desiredValue, int timeout=1500)
// {
// 	bool enableDrivePID = true;
// 	int prevError = 0;
// 	double totalError = 0;
// 	int count = 0;

// 	double kP = 0.75;
// 	double kI = 0.000575; 
// 	double kD = 3.3;
// 	double maxI = 500;
	
// 	int integralThreshold = 150;

// 	int time = 0;

// 	chassis_FR.tare_position();
// 	chassis_FL.tare_position();
// 	chassis_BR.tare_position();
// 	chassis_BL.tare_position();

// 	// inertial.tare_heading();

// 	con.clear();

// 	double initialValue = inertial.get_heading();
// 	if (initialValue > 180){
// 		initialValue = initialValue - 360;
// 	}

// 	while (enableDrivePID)
// 	{

// 		if (time > timeout){
// 			enableDrivePID = false;
// 		}

// 		// get position of all motors:
// 		int FRpos = chassis_FR.get_position();
// 		int FLpos = chassis_FL.get_position();
// 		int BRpos = chassis_BR.get_position();
// 		int BLpos = chassis_BL.get_position();

// 		double currentIMUValue = inertial.get_heading();
// 		if (currentIMUValue > 180){
// 			currentIMUValue = currentIMUValue - 360;
// 		}
// 		double headingCorrection = initialValue -currentIMUValue;
// 		headingCorrection = headingCorrection * 5;



// 		// get avg of motors:
// 		int currentValue = (FRpos + BRpos + FLpos + BLpos) / 4;

// 		// proportional
// 		double error = desiredValue - currentValue;

// 		// derivative
// 		int derivative = error - prevError;

// 		// integral
// 		if (abs(error) < integralThreshold)
// 		{
// 			totalError += error;
// 		}

// 		if (error > 0){
// 			totalError = min(totalError, maxI);
// 		}
// 		else{
// 			totalError = max(totalError, -maxI);
// 		}

// 		double speed = (error * kP + derivative * kD + totalError * kI);


// 		if (speed>127){
// 			speed = 127;
// 		}
// 		else if (speed < -127){
// 			speed = -127;
// 		}

// 		leftChassis.move(speed - headingCorrection);
// 		rightChassis.move(speed + headingCorrection);

// 		con.print(0,0, "error: %f", float(error));


// 		prevError = error;

// 		if (error < 10)
// 		{
// 			count++;
// 		}

// 		if (count > 20)
// 		{
// 			enableDrivePID = false;
// 		}

// 		delay(20);

// 		time++; //add one to time every cycle
		
// 	}

// 	chassis.move(0);
// }

// void turnPID(int desiredValue, int timeout=1500)
// {
// 	bool enableTurnPID = true;
// 	int prevError = 0;
// 	double totalError = 0;
// 	int count = 0;
// 	double position;
// 	double turnV;

// 	double kP = 8;
// 	double kI = 0.0001; 
// 	double kD = 30;
// 	double maxI = 500;

// 	int time = 0;
	
// 	int integralThreshold = 30;

// 	con.clear();

// 	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

// 	//inertial.tare_heading();


// 	position = inertial.get_heading();
// 	if (position > 180){
// 		position = ((360-position) * -1);
// 	}

// 	if ((desiredValue < 0) && (position > 0)){
// 		if ((position - desiredValue) >= 180){
// 			desiredValue = desiredValue + 360;
// 			position = inertial.get_heading();
// 			turnV = (position + desiredValue);
// 		}
// 		else {
// 			turnV = (abs(position) + abs(desiredValue));
// 		}
// 	}
// 	else if ((desiredValue > 0) && (position < 0)) {
// 		if ((desiredValue - position) >= 180){
// 			position = inertial.get_heading();
// 		}
// 		else {
// 			turnV = (position + desiredValue);
// 		}
// 	}

// 	// if (abs(turnV)<10){
// 	// 	//constants
// 	// }
// 	// else if (abs(turnV < 20)){
// 	// 	//constants
// 	// }
// 	// else if (abs(turnV < 30)){
// 	// 	//constants
// 	// }
// 	// else if (abs(turnV < 40)){
// 	// 	//constants
// 	// }
// 	// else if (abs(turnV < 50)){
// 	// 	//constants
// 	// }
// 	// else if (abs(turnV < 70)){
// 	// 	//constants
// 	// }
// 	// else if (abs(turnV <= 50)){
// 	// 	kP = 8;
// 	// 	kI = 0; 
// 	// 	kD = 0;
// 	// }
// 	// else if (abs(turnV <= 90)){
// 	// 	kP = 8;
// 	// 	kI = 0.0001; 
// 	// 	kD = 30;
// 	// }

// 	//tune distances
// 	//test intervals


// 	while (enableTurnPID)
// 	{
// 		if (time > timeout){
// 			enableTurnPID = false;
// 		}

// 		// get avg of motors:
// 		position = inertial.get_heading();
// 		if (position > 180){
// 			position = ((360-position) * -1);
// 		}

// 		if ((desiredValue < 0) && (position > 0)){
// 			if ((position - desiredValue) >= 180){
// 				desiredValue = desiredValue + 360;
// 				position = inertial.get_heading();
// 				turnV = (position + desiredValue);
// 			}
// 			else {
// 				turnV = (abs(position) + abs(desiredValue));
// 			}
// 		}
// 		else if ((desiredValue > 0) && (position < 0)) {
// 			if ((desiredValue - position) >= 180){
// 				position = inertial.get_heading();
// 			}
// 			else {
// 				turnV = (position + desiredValue); //for different constants for pid
// 			}
// 		}

// 		// proportional
// 		int error = desiredValue - position;

// 		// derivative
// 		int derivative = error - prevError;

// 		// integral
// 		if (abs(error) < integralThreshold)
// 		{
// 			totalError += error;
// 		}

// 		if (error > 0){
// 			totalError = min(totalError, maxI);
// 		}
// 		else{
// 			totalError = max(totalError, -maxI);
// 		}

// 		con.print(0,0, "error: %f", float(error));

// 		double speed = (error * kP + derivative * kD + totalError * kI);
// 		rightChassis.move(speed);
// 		leftChassis.move(-speed);

// 		prevError = error;

// 		if (error < 5)
// 		{
// 			count++;
// 		}

// 		if (count > 30)
// 		{
// 			enableTurnPID = false;
// 		}

// 		delay(20);
// 		time++;
// 	}

// 	chassis.move(0);
// }



void program1()
{
	
}

void program2(){

}

void program3()
{
	
}

void skipAutonomous()
{
	con.rumble("..........");
}
