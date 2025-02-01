#include "main.h"
#include "global.h"
#include <cmath>
#include <limits>
#include <chrono>
#include <thread>

using namespace pros;
using namespace glb;

//get rid of equals in chaining arcs if it dont work

// void timerTask(){
// 	auto startTime = std::chrono::steady_clock::now();

// 	while(true){
// 		//check time passed
// 		auto currentTime = std::chrono::steady_clock::now();
// 		auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
// 	}
// }

double calcPID(double error, double kP=0.99, double kI=0.05, double kD=2.4, double totalError=0, double prevError=0, double integralThreshold=30, double maxI=500) {
	// calculate integral
	if (abs(error) < integralThreshold)
	{
		totalError += error;
	}

	if (error > 0){
		totalError = std::min(totalError, maxI);
	}
	else{
		totalError = std::max(totalError, -maxI);
	}

    // calculate derivative
    float derivative = error - prevError;
    prevError = error;

    // calculate output
    double speed = (error * kP) + (totalError * kI) + (derivative * kD);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;
}

double ladyBrownPID(double error, double kP=5, double kI=0, double kD=0, double totalError=0, double prevError=0, double integralThreshold=30, double maxI=500) {
	// calculate integral
	if (abs(error) < integralThreshold)
	{
		totalError += error;
	}

	if (error > 0){
		totalError = std::min(totalError, maxI);
	}
	else{
		totalError = std::max(totalError, -maxI);
	}

    // calculate derivative
    float derivative = error - prevError;
    prevError = error;

    // calculate output
    double speed = (error * kP) + (totalError * kI) + (derivative * kD);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;
}

void drivePID(int desiredValue, int timeout=15000, int chainPos=0, bool autoclamp=false, int speed_percent=100)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	bool chain;
	double prevSpeed=0;
	double speed = 0;

	double kP = 0.7;
	double kI = 0.001; 
	double kD = 2.7;
	double maxI = 500;
	
	int integralThreshold = 150;

	int time = 0;

	FR.tare_position();
	FL.tare_position();
	RM.tare_position();
	LM.tare_position();
	BR.tare_position();
	BL.tare_position();

	// desiredValue = desiredValue * 2; 

	double currentValue; 

	// inertial.tare_heading();

	//con.clear();

	double initialValue = inertial.get_heading();
	if (initialValue > 180){
		initialValue = ((360-initialValue) * -1);
	}

	if (chainPos == 0){
		chain = false;
	}
	else {
		chain = true;
	}

	while (enableDrivePID)
	{

		if (time > timeout){
			enableDrivePID = false;
		}

		double currentIMUValue = inertial.get_heading();
		if (currentIMUValue > 180){
			currentIMUValue = ((360-currentIMUValue) * -1);
		}

		if ((initialValue < 0) && (currentIMUValue > 0)){
			if ((currentIMUValue - initialValue) >= 180){
				initialValue = initialValue + 360;
				currentIMUValue = inertial.get_heading();
				// turnV = (initialValue - position); 
			}
			else {
				// turnV = (abs(position) + abs(initialValue));
			}
		}
		else if ((initialValue > 0) && (currentIMUValue < 0)) {
			if ((initialValue - currentIMUValue) >= 180){
				currentIMUValue = inertial.get_heading();
				// turnV = abs(abs(position) - abs(initialValue));
			}
			else {
				// turnV = (abs(position) + initialValue); 
			}
		}
		else {
            // turnV = abs(abs(position) - abs(initialValue));
        }

		double headingError = initialValue -currentIMUValue;
		double headingCorrection = calcPID(headingError, 4);



		// get avg of motors:
		currentValue = (FR.get_position() + RM.get_position() + BR.get_position() + FL.get_position() + LM.get_position() + BL.get_position()) / 6;

		// proportional
		double error = desiredValue - currentValue;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < integralThreshold)
		{
			totalError += error;
		}

		if (error > 0){
			totalError = std::min(totalError, maxI);
		}
		else{
			totalError = std::max(totalError, -maxI);
		}
		prevSpeed = speed;
		speed = (error * kP + derivative * kD + totalError * kI);

		if (speed>127 * double(speed_percent)/100.0){
			speed = 127 * double(speed_percent)/100.0;
		}
		else if (speed < -127 * double(speed_percent)/100.0){
			speed = -127 * double(speed_percent)/100.0;
		}

		if(abs(currentValue)<50){
			if(abs(speed-prevSpeed)>4.5)
			{
				if(speed>prevSpeed)
				{
					speed=prevSpeed+4.5;
				}
				else if(speed<prevSpeed)
				{
					speed=prevSpeed-4.5;
				}
			}

		}

		leftChassis.move(speed + headingCorrection);
		rightChassis.move(speed - headingCorrection);

		con.print(0,0, "error: %f", float(error));


		prevError = error;

		if (abs(error) < 20)
		{
			count++;
		}

		if (count > 26)
		{
			enableDrivePID = false;
		}

		if (chain == true && abs(error) <= chainPos){
			enableDrivePID = false;
		}

		if (autoclamp==true && clampDistance.get() <= 190){
			mogo.set_value(true);
		}

		delay(20);

		time+=20; //add one to time every cycle
		
	}

	chassis.move(0);	
}


void drivePIDMogo(int desiredValue, int timeout=15000, int chainPos=0, int perc=100)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	bool chain;

	double kP = 0.7;
	double kI = 0.001; 
	double kD = 2.6;
	double maxI = 500;
	
	int integralThreshold = 150;

	int time = 0;

	FR.tare_position();
	FL.tare_position();
	RM.tare_position();
	LM.tare_position();
	BR.tare_position();
	BL.tare_position();

	// inertial.tare_heading();

	con.clear();

	double initialValue = inertial.get_heading();
	if (initialValue > 180){
		initialValue = ((360-initialValue) * -1);
	}

	if (chainPos == 0){
		chain = false;
	}
	else {
		chain = true;
	}

	while (enableDrivePID)
	{

		if (time > timeout){
			enableDrivePID = false;
		}

		// get position of all motors:
		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int LMpos = LM.get_position();
		int RMpos = RM.get_position();

		double currentIMUValue = inertial.get_heading();
		if (currentIMUValue > 180){
			currentIMUValue = ((360-currentIMUValue) * -1);
		}

		if ((initialValue < 0) && (currentIMUValue > 0)){
			if ((currentIMUValue - initialValue) >= 180){
				initialValue = initialValue + 360;
				currentIMUValue = inertial.get_heading();
				// turnV = (initialValue - position); 
			}
			else {
				// turnV = (abs(position) + abs(initialValue));
			}
		}
		else if ((initialValue > 0) && (currentIMUValue < 0)) {
			if ((initialValue - currentIMUValue) >= 180){
				currentIMUValue = inertial.get_heading();
				// turnV = abs(abs(position) - abs(initialValue));
			}
			else {
				// turnV = (abs(position) + initialValue); 
			}
		}
		else {
            // turnV = abs(abs(position) - abs(initialValue));
        }
		
		double headingError = initialValue - currentIMUValue;
		double headingCorrection = calcPID(headingError, 4);



		// get avg of motors:
		int currentValue = (FRpos + LMpos + BRpos + FLpos + BLpos + RMpos) / 6;

		// proportional
		double error = desiredValue - currentValue;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < integralThreshold)
		{
			totalError += error;
		}

		if (error > 0){
			totalError = std::min(totalError, maxI);
		}
		else{
			totalError = std::max(totalError, -maxI);
		}

		double speed = (error * kP + derivative * kD + totalError * kI);


		if (speed>127 * perc/100){
			speed = 127 * perc/100;
		} 
		else if (speed < -127 * perc/100){
			speed = -127 * perc/100;
		}

		leftChassis.move(speed + headingCorrection);
		rightChassis.move(speed - headingCorrection);

		con.print(0,0, "error: %f", float(error));


		prevError = error;

		if (error < 20)
		{
			count++;
		}

		if (count > 20)
		{
			enableDrivePID = false;
		}

		if (chain == true && abs(error) <= chainPos){
			enableDrivePID = false;
		}

		delay(20);

		time+=20; //add one to time every cycle
		
	}

	chassis.move(0);
}

void turnPID(int desiredValue, int timeout=15000, bool powerFunc = true, int chainPos=0, double trueTheta=0)
{
	bool chain;
	
	if (chainPos == 0){
		chain = false;
	}
	else {
		chain = true;
		trueTheta = desiredValue;
		desiredValue = desiredValue + chainPos;
		if (desiredValue > 180){
			desiredValue = ((360-desiredValue) * -1);
		}
	}
	
	bool enableTurnPID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	double position;
	double turnV;

	double kP = 5;
	double kI = 0.001; 
	double kD = 18.9;
	double maxI = 500;

	int time = 0;
	
	int integralThreshold = 30;

	con.clear();

	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	//inertial.tare_heading();


	position = inertial.get_heading();
	if (position > 180){
		position = ((360-position) * -1);
	}

	if ((desiredValue < 0) && (position > 0)){
		if ((position - desiredValue) >= 180){
			desiredValue = desiredValue + 360;
			position = inertial.get_heading();
			turnV = (desiredValue - position); 
		}
		else {
			turnV = (abs(position) + abs(desiredValue));
		}
	}
	else if ((desiredValue > 0) && (position < 0)) {
		if ((desiredValue - position) >= 180){
			position = inertial.get_heading();
			turnV = abs(abs(position) - abs(desiredValue));
		}
		else {
			turnV = (abs(position) + desiredValue); 
		}
	}
	else {
		turnV = abs(abs(position) - abs(desiredValue));
	}

	if (powerFunc == true){
	//   y =    a   +          bx            +           cx^2                  +                dx^3                +                fx^4
		kD = 2.5706+(0.583313*abs(turnV))+(-0.00847392*pow(abs(turnV), 2))+(0.0000559881*pow(abs(turnV), 3))+(-0.000000129618*pow(abs(turnV), 4));
	}


	while (enableTurnPID)
	{
		if (time > timeout){
			enableTurnPID = false;
		}

		// get avg of motors:
		position = inertial.get_heading();
		if (position > 180){ //make only > if not working
			position = ((360-position) * -1);
		}

		if ((desiredValue < 0) && (position > 0)){
			if ((position - desiredValue) >= 180){
				desiredValue = desiredValue + 360;
				position = inertial.get_heading();
				turnV = (desiredValue - position); 
			}
			else {
				turnV = (abs(position) + abs(desiredValue));
			}
		}
		else if ((desiredValue > 0) && (position < 0)) {
			if ((desiredValue - position) >= 180){
				position = inertial.get_heading();
				turnV = abs(abs(position) - abs(desiredValue));
			}
			else {
				turnV = (abs(position) + desiredValue); 
			}
		}
		else {
            turnV = abs(abs(position) - abs(desiredValue));
        }

		// proportional
		int error = desiredValue - position;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < integralThreshold)
		{
			totalError += error;
		}

		if (error > 0){
			totalError = std::min(totalError, maxI);
		}
		else{
			totalError = std::max(totalError, -maxI);
		}


		double speed = (error * kP + derivative * kD + totalError * kI);
		rightChassis.move(-speed);
		leftChassis.move(speed);

		con.print(0,0, "error: %f", float(error));

		prevError = error;

		if (abs(error) < 5)
		{
			count++;
		}

		if (count > 10)
		{
			enableTurnPID = false;
		}

		if (chain == true && position >= trueTheta){
			enableTurnPID = false;
		}

		delay(20);
		time = time + 20;
	}

	chassis.move(0);
}

void turnPIDMogo(int desiredValue, int timeout=15000, bool powerFunc = true, int chainPos=0, double trueTheta=0)
{
	bool chain;
	
	if (chainPos == 0){
		chain = false;
	}
	else {
		chain = true;
		trueTheta = desiredValue;
		desiredValue = desiredValue + chainPos;
		if (desiredValue > 180){
			desiredValue = ((360-desiredValue) * -1);
		}
	}
	
	bool enableTurnPID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	double position;
	double turnV;

	double kP = 5;
	double kI = 0.001; 
	double kD = 22.5;
	double maxI = 500;

	int time = 0;
	
	int integralThreshold = 30;

	con.clear();

	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	//inertial.tare_heading();


	position = inertial.get_heading();
	if (position > 180){
		position = ((360-position) * -1);
	}

	if ((desiredValue < 0) && (position > 0)){
		if ((position - desiredValue) >= 180){
			desiredValue = desiredValue + 360;
			position = inertial.get_heading();
			turnV = (desiredValue - position); 
		}
		else {
			turnV = (abs(position) + abs(desiredValue));
		}
	}
	else if ((desiredValue > 0) && (position < 0)) {
		if ((desiredValue - position) >= 180){
			position = inertial.get_heading();
			turnV = abs(abs(position) - abs(desiredValue));
		}
		else {
			turnV = (abs(position) + desiredValue); 
		}
	}
	else {
		turnV = abs(abs(position) - abs(desiredValue));
	}

	if (powerFunc == true){
	//   y =    a   +          bx            +           cx^2                  +                dx^3                +                fx^4
		kD = 9.82234+(0.208006*abs(turnV))+(-0.00127772*pow(abs(turnV), 2))+(0.0000087524*pow(abs(turnV), 3))+(-0.0000000258161*pow(abs(turnV), 4));
	}


	while (enableTurnPID)
	{
		if (time > timeout){
			enableTurnPID = false;
		}

		// get avg of motors:
		position = inertial.get_heading();
		if (position > 180){ //make only > if not working
			position = ((360-position) * -1);
		}

		if ((desiredValue < 0) && (position > 0)){
			if ((position - desiredValue) >= 180){
				desiredValue = desiredValue + 360;
				position = inertial.get_heading();
				turnV = (desiredValue - position); 
			}
			else {
				turnV = (abs(position) + abs(desiredValue));
			}
		}
		else if ((desiredValue > 0) && (position < 0)) {
			if ((desiredValue - position) >= 180){
				position = inertial.get_heading();
				turnV = abs(abs(position) - abs(desiredValue));
			}
			else {
				turnV = (abs(position) + desiredValue); 
			}
		}
		else {
            turnV = abs(abs(position) - abs(desiredValue));
        }

		// proportional
		int error = desiredValue - position;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < integralThreshold)
		{
			totalError += error;
		}

		if (error > 0){
			totalError = std::min(totalError, maxI);
		}
		else{
			totalError = std::max(totalError, -maxI);
		}


		double speed = (error * kP + derivative * kD + totalError * kI);
		rightChassis.move(-speed);
		leftChassis.move(speed);

		con.print(0,0, "error: %f", float(error));

		prevError = error;

		if (abs(error) < 5)
		{
			count++;
		}

		if (count > 10)
		{
			enableTurnPID = false;
		}

		if (chain == true && position >= trueTheta){
			enableTurnPID = false;
		}

		delay(20);
		time = time + 20;
	}

	chassis.move(0);
}


//arc turn below - motors and inertial was blowing up so i had to comment (sorryyy):
float totalError;
float prevError;
float kP;
float kI;
float kD;
int integralThreshold = 30;
double maxI = 500;

float calculatePID(float error){

	kP = 0.7;
	kI = 0.001; 
	kD = 2.7;
	
	// calculate integral
	if (abs(error) < integralThreshold)
	{
		totalError += error;
	}

    // calculate derivative
    float derivative = error - prevError;
    prevError = error;

    // calculate output
    double speed = (error * kP) + (totalError * kI) + (derivative * kD);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;

}

float totalError2;
float prevError2;
float kP2;
float kI2;
float kD2;
int integralThreshold2 = 30;
double maxI2 = 500;

float calculatePID2(float error){

	kP2 = 0.7;
	kI2 = 0.001; 
	kD2 = 2.7;
	
	// calculate integral
	if (abs(error) < integralThreshold2)
	{
		totalError2 += error;
	}

    // calculate derivative
    float derivative = error - prevError2;
    prevError2 = error;

    // calculate output
    double speed = (error * kP2) + (totalError2 * kI2) + (derivative * kD2);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;

}


void leftArc(double radius, double centralDegreeTheta, int chainPos=0, int timeout=15000, std::string createTask="off", int taskStart=0, int taskEnd=0, double trueTheta=0){
	bool chain;

	double rightArc = ((centralDegreeTheta+chainPos) / 360)*2*M_PI*(radius + 275);
	double leftArc = ((centralDegreeTheta+chainPos) / 360)*2*M_PI*(radius - 275);

	double speedProp = rightArc/leftArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	bool taskStarted = false;
	bool taskEnded = false;

	double init_heading = inertial.get_heading(); 

	if (chainPos == 0){
		chain = false;
	}
	else {
		chain = true;
		trueTheta = centralDegreeTheta;
		centralDegreeTheta = centralDegreeTheta + chainPos;
	}

	while(1){
		
		if (time > timeout){
			break;
		}

		if(init_heading > 180){
            init_heading = init_heading - 360;
        }

		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int MRpos = RM.get_position();
		int MLpos = LM.get_position();

		int currentRightPosition = (FRpos + BRpos + MRpos)/3;
		int currentLeftPosition = (FLpos + BLpos + MLpos)/3;

		int right_error = rightArc - currentRightPosition;
		int left_error = leftArc - currentLeftPosition;

		double leftcorrect = -(currentLeftPosition * 360) / (2*M_PI*(radius)); 

		double heading = inertial.get_heading(); 
		if (heading > 180){
            heading = heading - 360;
        }

		if(((init_heading + leftcorrect)< 0) && (heading > 0)){
            if((heading - (init_heading + leftcorrect)) >= 180){
                leftcorrect = leftcorrect + 360;
                heading = inertial.get_heading();
            } 
        } else if (((init_heading + leftcorrect) > 0) && (heading < 0)){
            if(((init_heading + leftcorrect) - heading) >= 180){
            	heading = inertial.get_heading();
            }
        }

		int fix = int(init_heading + leftcorrect-heading);
		fix = fix*5;
		leftChassis.move(calculatePID(left_error) + fix);
		rightChassis.move(calculatePID2(right_error) - fix); //might need to add calcpid2

		if ((abs(left_error) <= 50) && (abs(right_error) <= 50)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs((init_heading - heading)) >= trueTheta){ //might need to add calcpid2 back ON THE RIGHT SIDE
			break;
		}

		// if (time >= taskStart && taskStarted == false){
		// 	if (createTask == "frontWings"){
		// 		frontLeftWing.set_value(true);
		// 		frontRightWing.set_value(true);
		// 		taskStarted == true;
		// 	}
		// 	else if (createTask == "backWings"){
		// 		backLeftWing.set_value(true);
		// 		backRightWing.set_value(true);
		// 		taskStarted == true;
		// 	}
		// 	else if (createTask == "reverseIntake"){
		// 		intake.move(127);
		// 		taskStarted == true;
		// 	}
		// 	else if (createTask == "forwardIntake"){
		// 		intake.move(-127);
		// 		taskStarted == true;
		// 	}
		// }
		// if (time >= taskEnd && taskEnded == false){
		// 	if (createTask == "frontWings"){
		// 		frontLeftWing.set_value(false);
		// 		frontRightWing.set_value(false);
		// 		taskEnded == true;
		// 	}
		// 	else if (createTask == "backWings"){
		// 		backLeftWing.set_value(false);
		// 		backRightWing.set_value(false);
		// 		taskEnded == true;
		// 	}
		// 	else if (createTask == "reverseIntake"){
		// 		intake.move(0);
		// 		taskEnded == true;
		// 	}
		// 	else if (createTask == "forwardIntake"){
		// 		intake.move(0);
		// 		taskEnded == true;
		// 	}
		// }		

		delay(20);
		time = time+20;

	}
	chassis.move(0);
}

void rightArc(double radius, double centralDegreeTheta, int chainPos=0, int timeout=15000, std::string createTask="off", int taskStart=0, int taskEnd=0, double trueTheta=0){

	bool chain;

	double rightArc = ((centralDegreeTheta+chainPos) / 360)*2*M_PI*(radius - 275);
	double leftArc = ((centralDegreeTheta+chainPos) / 360)*2*M_PI*(radius + 275);

	//double speedProp = leftArc/rightArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	
	bool taskStarted = false;
	bool taskEnded = false;
	
	// con.clear();

	double init_heading = inertial.get_heading();
	if (chainPos == 0){
		chain = false;
	}
	else {
		chain = true;
		trueTheta = centralDegreeTheta;
		centralDegreeTheta = centralDegreeTheta + chainPos;
	}

	while(1){

		if(init_heading > 180){
            init_heading = init_heading - 360;
        }

		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int MRpos = RM.get_position();
		int MLpos = LM.get_position();

		int currentRightPosition = (FRpos + BRpos + MRpos)/3;
		int currentLeftPosition = (FLpos + BLpos + MLpos)/3;
		int right_error = rightArc - currentRightPosition;
		int left_error = leftArc - currentLeftPosition;

		double rightcorrect = (currentRightPosition * 360) / (2*M_PI*(radius)); 

		//con.print(0,0, "imu: %f", float(inertial.get_heading()));

		double heading = inertial.get_heading(); 
		if (heading > 180){
            heading = heading - 360;
        }

        if(((init_heading + rightcorrect) < 0) && (heading > 0)){
            if((heading - (init_heading + rightcorrect)) >= 180){
                init_heading = init_heading + 360;
                heading = inertial.get_heading();
            } 
        } else if (((init_heading + rightcorrect)> 0) && (heading < 0)){
            if(((init_heading + rightcorrect) - heading) >= 180){
            heading = inertial.get_heading();
            }
        } 

		int fix = int(init_heading + rightcorrect - heading);
		fix = fix*5;
		leftChassis.move(calculatePID(left_error) - fix);
		rightChassis.move(calculatePID2(right_error) + fix); //might need to add calcpid2
		

		//con.print(0,0, "rc: %f", float(rightArc));

		if ((abs(left_error) <= 50) && (abs(right_error) <= 50)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs((init_heading - heading)) >= trueTheta){ //might need to add calcpid2 back ON THE RIGHT SIDE
			break;
		}

		// if (time >= taskStart && taskStarted == false){
		// 	if (createTask == "frontWings"){
		// 		frontLeftWing.set_value(true);
		// 		frontRightWing.set_value(true);
		// 		taskStarted == true;
		// 	}
		// 	else if (createTask == "backWings"){
		// 		backLeftWing.set_value(true);
		// 		backRightWing.set_value(true);
		// 		taskStarted == true;
		// 	}
		// 	else if (createTask == "reverseIntake"){
		// 		intake.move(127);
		// 		taskStarted == true;
		// 	}
		// 	else if (createTask == "forwardIntake"){
		// 		intake.move(-127);
		// 		taskStarted == true;
		// 	}
		// }
		// if (time >= taskEnd && taskEnded == false){
		// 	if (createTask == "frontWings"){
		// 		frontLeftWing.set_value(false);
		// 		frontRightWing.set_value(false);
		// 		taskEnded == true;
		// 	}
		// 	else if (createTask == "backWings"){
		// 		backLeftWing.set_value(false);
		// 		backRightWing.set_value(false);
		// 		taskEnded == true;
		// 	}
		// 	else if (createTask == "reverseIntake"){
		// 		intake.move(0);
		// 		taskEnded == true;
		// 	}
		// 	else if (createTask == "forwardIntake"){
		// 		intake.move(0);
		// 		taskEnded == true;
		// 	}
		// }

		delay(20);
		time = time+20;

	}
	chassis.move(0);
}

class Point {
	public:
		float x;
		float y;

		float theta = std::numeric_limits<float>::quiet_NaN(); //theta is initialized at NaN (quiet so it doesnt blow everything up) so it doesnt need to be updated at initialization, and then when its finally given a value we can check if it was updated by checking if it is a real number

		//constructor - automatically ran when the class is used
        Point(float x, float y, float theta = std::numeric_limits<float>::quiet_NaN())
            : x(x),
              y(y),
              theta(theta) {}
		
		//function to calculate the distance between two points
		float distanceTo(const Point& other) const{
			float deltaX = x - other.x;
			float deltaY = y - other.y;

			return sqrt(deltaX*deltaX + deltaY*deltaY);
		}

		//function for converting degrees to radians
		static float degreesToRadians(float degrees){
			return degrees*M_PI/180.0;
		}

		//function for calculated the difference in error for two different points
		float angleError(const Point& other) const{
			return other.theta - theta;
		}
};

// void boomerang(float x, float y, float theta, float dlead){
// 	float linearError;
// 	float linearPower;
// 	float angularError;
// 	float angularPower;
// 	// calculate target pose in standard form
// 	Point target(x, y, Point::degreesToRadians(theta));
// }


void redRushSide()
{
	//red rush
	drivePID(-1500, 35);
	turnPID(-33, 1500, false);
	drivePID(-550, 30, 0, true);
	mogo.set_value(true);
	delay(50);
	drivePIDMogo(600,1500,10);
	turnPIDMogo(0, 1500, false);
	drivePIDMogo(750);
	intake.move(127);
	delay(750);
	turnPIDMogo(-179);
	mogo.set_value(false);
	turnPID(135);
	intake.move(60);
	drivePID(800);
	intake.move(0);
	turnPID(-90, 1500, false);
	drivePID(-500);
	drivePID(-400, 1500, 0, true, 50);
	mogo.set_value(true);
	delay(50);
	intake.move(127);
}

void blueRushSide(){
	//blue rush
	drivePID(-1500, 35);
	turnPID(33, 1500, false);
	drivePID(-550, 30, 0, true);
	mogo.set_value(true);
	delay(50);
	drivePIDMogo(600,1500,10);
	turnPIDMogo(0, 1500, false);
	drivePIDMogo(750);
	intake.move(127);
	delay(750);
	turnPIDMogo(179);
	mogo.set_value(false);
	turnPID(-135);
	intake.move(60);
	drivePID(800);
	intake.move(0);
	turnPID(90, 1500, false);
	drivePID(-500);
	drivePID(-400, 1500, 0, true, 50);
	mogo.set_value(true);
	delay(50);
	intake.move(127);
}

void redRingSide(){
	//red ring
	drivePID(-850, 30);
	turnPID(-31);
	drivePID(-400);
	mogo.set_value(true);
	delay(50);
	drivePID(-200);
	intake.move(127);
	delay(500);
	turnPIDMogo(88);
	drivePID(900);
	turnPID(170);
	drivePIDMogo(700);
	drivePIDMogo(-700);
	turnPIDMogo(150);
	drivePIDMogo(800);
	drivePIDMogo(-1500, 2000);
	turnPID(30);
	drivePID(-2000);
}

void blueRingSide(){
	//blue ring
	drivePID(-850, 30);
	turnPID(-31);
	drivePID(-400);
	mogo.set_value(true);
	delay(50);
	drivePID(-200);
	intake.move(127);
	delay(500);
	turnPIDMogo(88);
	drivePID(900);
	turnPID(170);
	drivePIDMogo(700);
	drivePIDMogo(-700);
	turnPIDMogo(150);
	drivePIDMogo(800);
	drivePIDMogo(-1500, 2000);
	turnPID(30);
	drivePID(-2000);
}

void skillsAuto()
{
	// //skills auto
	intake.move(127);
	delay(500);
	drivePID(600);
	turnPID(90);
	drivePID(-500);
	drivePID(-300, 1500, 0, true, 50);
	mogo.set_value(true);
	delay(700);
	turnPIDMogo(-20, 1500, false);
	drivePIDMogo(900);
	turnPIDMogo(-47);
	drivePIDMogo(1800);
	delay(300);
	drivePIDMogo(-300);
	turnPIDMogo(-175);
	delay(100);
	drivePIDMogo(500);
	delay(10);
	drivePIDMogo(1100);
	delay(10);
	drivePIDMogo(800);
	drivePIDMogo(-600);
	turnPIDMogo(-90, 1500, false);
	drivePIDMogo(500);
	drivePIDMogo(-500);
	turnPID(45);
	mogo.set_value(false);
	drivePIDMogo(-700);
	drivePID(900);
	turnPID(90);
	drivePID(4000);
	turnPID(135);
	drivePID(1100);
	drivePID(-1000);
	turnPID(-180);
	drivePID(-2500);
	turnPID(-45);
	drivePID(1000);
	turnPID(45);
	drivePID(1000);
	//this is where Larry started. none tuned just skeleton code
	drivePID(-2500);
	turnPID(0);
	drivePID(2000);
	turnPID(-45);
	drivePID(2500);
	turnPID(-120);
	drivePID(-1500);
	drivePID(1500);
	turnPID(-90);
	drivePID(2000);
	turnPID(-30);
	drivePID(2000);
	intake.move(127);
	delay(500);
	drivePID(600);
	turnPID(90);
	drivePID(-500);
	drivePID(-300, 1500, 0, true, 50);
	mogo.set_value(true);
	delay(700);
	turnPIDMogo(-20, 1500, false);
	drivePIDMogo(900);
	turnPIDMogo(-47);
	drivePIDMogo(1800);
	delay(300);
	drivePIDMogo(-300);
	turnPIDMogo(-175);
	delay(100);
	drivePIDMogo(500);
	delay(10);
	drivePIDMogo(1100);
	delay(10);
	drivePIDMogo(800);
	drivePIDMogo(-600);
	turnPIDMogo(-90, 1500, false);
	drivePIDMogo(500);
	drivePIDMogo(-500);
	turnPID(45);
	mogo.set_value(false);
	drivePIDMogo(-700);
	drivePID(900);
	turnPID(90);
	drivePID(4000);
	turnPID(135);
	drivePID(1100);
	drivePID(-1000);
	turnPID(-180);
	drivePID(-2500);
	turnPID(-45);
	drivePID(1000);
	turnPID(45);
	drivePID(1000);
}


void skipAutonomous()
{
	//skills goes here
	
// 	ladyBrown.move(127);
//     delay(200);
//     ladyBrown.move(0);
//     intake.move(127);
//     delay(500);
//     ladyBrown.move(-127);
//     delay(500);
//     intake.move(-80);
//     delay(100);
//     ladyBrown.move(0);
//     drivePID(600);
//     turnPID(-90);

//     drivePID(-820, 3300, 0, true,40);
//     mogo.set_value(true);
//     drivePIDMogo(-200);

//     turnPIDMogo(0);
//     intake.move(127);
//     drivePIDMogo(1090, 2000, 0, 70);
//     delay(100);
//     turnPIDMogo(90);
//     drivePIDMogo(950);
    
//     turnPIDMogo(0);
//     drivePIDMogo(950);




//     turnPIDMogo(90);
//     lbPID = true;
//     intake.move(127);
//     drivePIDMogo(720, 600);
//     delay(1000);
//     intake.move(0);
//     lbPID = false;
//     ladyBrown.move(127);
//     delay(550);
//     ladyBrown.move(-127);
//     drivePIDMogo(-500);
//     ladyBrown.move(0);
//     turnPIDMogo(0);
//     intake.move(-127);
//     delay(300);
//     intake.move(127);
//     drivePIDMogo(1200);
//     turnPIDMogo(35);
//     drivePIDMogo(730,3000,0,50);
//     turnPIDMogo(-53);
//     drivePIDMogo(200);
//     turnPIDMogo(-15);
//     drivePIDMogo(500);
//     turnPIDMogo(-130);
// //  // doinker.set_value(false);
//     intake.move(-80);
    
//     mogo.set_value(false);
//     drivePIDMogo(-500, 1500);
//     delay(1000);
//     drivePID(300, 1500, 0, false, 40);
//     mogo.set_value(true);
//     drivePID(-250, 1500, 0, false, 70);



//     intake.move(127);
//     mogo.set_value(false);
//     turnPID(-140);
//     drivePID(2100);
//     intake.move(0);
//     turnPID(140);
//     drivePID(-1600, 15000, 0, false, 40);
//     mogo.set_value(true);
//     intake.move(127);
//     turnPIDMogo(-135);
//     drivePIDMogo(1500);
//     turnPIDMogo(-90);
//     drivePIDMogo(1180);
//     turnPIDMogo(180);
//     drivePIDMogo(2000, 15000, 70);
//     drivePIDMogo(1500, 15000, 0, 60);

//     turnPIDMogo(45);
//     drivePIDMogo(-500);
//     mogo.set_value(false);
//     intake.move(-80);
//     delay(300);
//     drivePID(750);
//     intake.move(0);
//     turnPID(0);
//     drivePID(2400, 15000, 50);
//     turnPID(47);
//     drivePID(3300);
//     turnPID(92);
//     drivePID(-1500,1500,0,false,70);
//     mogo.set_value(true);
//     turnPIDMogo(110);
//     mogo.set_value(false);
//     drivePID(-1100);
    
//    
//     drivePID(200);
//     turnPID(180);
//     drivePID(5000);
    
    
//     turnPID(-90);
//     drivePID(-1200,1500,0,false,60);
//     mogo.set_value(true);
//     turnPIDMogo(90);
//     drivePIDMogo(4000);
//	   drivePIDMogo(1000,2000,0,60)
//     //leftArc(600, -135);
//     turnPID(-30);
    
//     mogo.set_value(false);
//     ladyBrown.move(127);
//     drivePID(1500, 15000, 100);
//     turnPID(135, 15000, true, 10);
//     drivePID(-3000);
	
	
	//ring side (5 so far)
	// drivePID(-1000);
	// drivePID(-200, 1500, 0, true, 50);
	// mogo.set_value(true);
	// intake.move(127);
	// delay(100);
	// turnPIDMogo(115);
	// drivePIDMogo(1100);
	// delay(150);
	// turnPIDMogo(75);
	// drivePIDMogo(500);
	// delay(100);
	// drivePIDMogo(-1000);
	// turnPIDMogo(40);
	// drivePIDMogo(800);
	// turnPIDMogo(-5);
	// drivePIDMogo(2400);





	// red ring
	// ladyBrown.move(127);
	// delay(500);
	// ladyBrown.move(-127);
	// delay(500);
	// ladyBrown.move(0);
	// turnPID(20);
	// drivePID(-1800,2000,0,0,60);
	// mogo.set_value(true);
	// intake.move(127);
	// turnPIDMogo(165);
	// drivePIDMogo(800);
	// turnPIDMogo(142);
	// drivePIDMogo(400, 15000, 10);
	// turnPIDMogo(30);
	// drivePIDMogo(300);
	// turnPIDMogo(-20);
	// intakeLift.set_value(true);
	// drivePIDMogo(1800);
	// drivePIDMogo(400, 15000, 0, 20);
	// intakeLift.set_value(false);
	// delay(300);
	// drivePIDMogo(-200);
 	// turnPIDMogo(-150);
	// drivePIDMogo(700);
	// ladyBrown.move(127);



	//red rush
	// drivePID(1550, 3000, 200);
	// turnPID(-10);
	// doinker.set_value(true);
	// delay(300);
	// leftArc(700,-40);
	// doinker.set_value(false);
	// delay(200);
	// turnPID(160);
	// drivePID(-1000, 15000, 0, true, 40);
	// mogo.set_value(true);
	// delay(200);
	// intake.move(127);
	// turnPIDMogo(-160);
	// drivePIDMogo(650);
	// delay(1000);
	// mogo.set_value(false);
	// delay(200);
	// turnPIDMogo(90);
	// drivePID(-1000,5000,0,true, 40);
	// mogo.set_value(true);
	// turnPIDMogo(-132);
	// intakeLift.set_value(true);
	// drivePIDMogo(1000);
	// drivePIDMogo(100,10000,0,20);
	// drivePIDMogo(-1100);
	// turnPIDMogo(-45);
	// drivePIDMogo(400);
	// ladyBrown.move(127);
	// delay(600);
	// ladyBrown.move(0);



	// //Blue rush
	
	drivePID(1500);
	intake.move(100);
	
	doinker.set_value(true);
	intake.move(60);
	drivePID(200);
	intake.move(0);
	drivePID(-650);
	doinker.set_value(false);
	delay(200);
	turnPID(160);
	drivePID(-800, 15000, 0, true, 30);
	mogo.set_value(true);
	delay(200);
	intake.move(127);
	delay(1000);
	mogo.set_value(false);
	drivePID(800);
	turnPID(-110);
	drivePID(-1000, 15000, 0, true, 30);
	mogo.set_value(true);
	turnPIDMogo(115);
	intakeLift.set_value(true);
	drivePIDMogo(1000);
	drivePIDMogo(100,10000,0,20);
	drivePIDMogo(-1100);
	turnPIDMogo(-45);
	drivePIDMogo(400);
	ladyBrown.move(127);
	delay(600);
	ladyBrown.move(0);

	// delay(1000);
	// mogo.set_value(false);
	// delay(200);
	// turnPIDMogo(-90);
	// drivePID(-1000,5000,0,true, 40);
	// mogo.set_value(true);
	// turnPIDMogo(132);
	// intakeLift.set_value(true);
	// drivePIDMogo(1000);
	// drivePIDMogo(100,10000,0,20);
	// drivePIDMogo(-1100);
	// turnPIDMogo(45);
	// drivePIDMogo(400);
	// ladyBrown.move(127);
	// delay(600);
	// ladyBrown.move(0);


}
