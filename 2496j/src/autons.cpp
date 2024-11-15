#include "main.h"
#include "global.h"
#include <cmath>
#include <limits>
#include <chrono>
#include <thread>

using namespace pros;
using namespace glb;

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

void driveSPID(int desiredValue, int timeout=1500, int chainSpeed=0)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	bool chain;

	double kP = 0.4;
	double kI = 0.001; 
	double kD = 2.8;
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
		initialValue = initialValue - 360;
	}

	if (chainSpeed == 0){
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
			currentIMUValue = currentIMUValue - 360;
		}

		if ((initialValue < 0) && (currentIMUValue > 0)){
			if ((currentIMUValue - initialValue) >= 180){
				initialValue = initialValue + 360;
				currentIMUValue = inertial.get_heading();
			}
		}

		else if ((initialValue > 0) && (currentIMUValue < 0)) {
			if ((initialValue - currentIMUValue) >= 180){
				currentIMUValue = inertial.get_heading();
			}
		}

		double headingError = initialValue -currentIMUValue;
		double headingCorrection = calcPID(headingError);



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


		if (speed>127){
			speed = 127;
		}
		else if (speed < -127){
			speed = -127;
		}

		leftChassis.move(speed + headingCorrection);
		rightChassis.move(speed - headingCorrection);

		con.print(0,0, "error: %f", float(error));


		prevError = error;

		if (error < 10)
		{
			count++;
		}

		if (count > 20)
		{
			enableDrivePID = false;
		}

		if (chain == true && abs(speed) <= chainSpeed){
			enableDrivePID = false;
		}

		delay(20);

		time++; //add one to time every cycle
		
	}

	chassis.move(0);
}


void drivePID(int desiredValue, int timeout=1500, int chainSpeed=0, bool autoclamp=false)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	bool chain;

	double kP = 0.7;
	double kI = 0.001; 
	double kD = 2.8;
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
		initialValue = initialValue - 360;
	}

	if (chainSpeed == 0){
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
			currentIMUValue = currentIMUValue - 360;
		}

		if ((initialValue < 0) && (currentIMUValue > 0)){
			if ((currentIMUValue - initialValue) >= 180){
				initialValue = initialValue + 360;
				currentIMUValue = inertial.get_heading();
			}
		}

		else if ((initialValue > 0) && (currentIMUValue < 0)) {
			if ((initialValue - currentIMUValue) >= 180){
				currentIMUValue = inertial.get_heading();
			}
		}

		double headingError = initialValue -currentIMUValue;
		double headingCorrection = calcPID(headingError);



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


		if (speed>127){
			speed = 127;
		}
		else if (speed < -127){
			speed = -127;
		}

		leftChassis.move(speed + headingCorrection);
		rightChassis.move(speed - headingCorrection);

		con.print(0,0, "error: %f", float(error));


		prevError = error;

		if (abs(error) < 10)
		{
			count++;
		}

		if (count > 20)
		{
			enableDrivePID = false;
		}

		if (chain == true && abs(speed) <= chainSpeed){
			enableDrivePID = false;
		}

		if (autoclamp==true && clampDistance.get() <= 190){
			mogo.set_value(true);
		}

		delay(20);

		time++; //add one to time every cycle
		
	}

	chassis.move(0);
}

void drivePIDMogo(int desiredValue, int timeout=1500, int chainSpeed=0)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	bool chain;

	double kP = 0.7;
	double kI = 0.001; 
	double kD = 3.3;
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
		initialValue = initialValue - 360;
	}

	if (chainSpeed == 0){
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
			currentIMUValue = currentIMUValue - 360;
		}

		if ((initialValue < 0) && (currentIMUValue > 0)){
			if ((currentIMUValue - initialValue) >= 180){
				initialValue = initialValue + 360;
				currentIMUValue = inertial.get_heading();
			}
		}

		else if ((initialValue > 0) && (currentIMUValue < 0)) {
			if ((initialValue - currentIMUValue) >= 180){
				currentIMUValue = inertial.get_heading();
			}
		}
		
		double headingError = initialValue -currentIMUValue;
		double headingCorrection = calcPID(headingError);



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


		if (speed>127){
			speed = 127;
		}
		else if (speed < -127){
			speed = -127;
		}

		leftChassis.move(speed + headingCorrection);
		rightChassis.move(speed - headingCorrection);

		con.print(0,0, "error: %f", float(error));


		prevError = error;

		if (error < 10)
		{
			count++;
		}

		if (count > 13)
		{
			enableDrivePID = false;
		}

		if (chain == true && abs(speed) <= chainSpeed){
			enableDrivePID = false;
		}

		delay(20);

		time++; //add one to time every cycle
		
	}

	chassis.move(0);
}

void turnPID(int desiredValue, int timeout=1500, bool powerFunc = true)
{
	bool enableTurnPID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	double position;
	double turnV;

	double kP = 5;
	double kI = 0.001; 
	double kD = 25;
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
			turnV = (position + desiredValue);
		}
		else {
			turnV = (abs(position) + abs(desiredValue));
		}
	}
	else if ((desiredValue > 0) && (position < 0)) {
		if ((desiredValue - position) >= 180){
			position = inertial.get_heading();
			turnV = 360 - desiredValue - abs(position);
		}
		else {
			turnV = (position + desiredValue);
		}
	}

	if (powerFunc == true){
	//   y =    a   +          bx            +           cx^2                  +                dx^3                +                fx^4
		kD = 0.769503+(0.279153*abs(desiredValue))+(-0.00232581*pow(abs(desiredValue), 2))+(0.0000156359*pow(abs(desiredValue), 3))+(-0.0000000398214*pow(abs(desiredValue), 4));
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
				turnV = (position + desiredValue);
			}
			else {
				turnV = (abs(position) + abs(desiredValue));
			}
		}
		else if ((desiredValue > 0) && (position < 0)) {
			if ((desiredValue - position) >= 180){
				position = inertial.get_heading();
				turnV = 360 - desiredValue - abs(position);
			}
			else {
				turnV = (position + desiredValue); //for different constants for pid
			}
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

		delay(20);
		time = time + 20;
	}

	chassis.move(0);
}


void turnPIDMogo(int desiredValue, int timeout=1500, bool powerFunc=true)
{
	bool enableTurnPID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	double position;
	double turnV;

	double kP = 5;
	double kI = 0.001; 
	double kD = 25;
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
			turnV = (position + desiredValue);
		}
		else {
			turnV = (abs(position) + abs(desiredValue));
		}
	}
	else if ((desiredValue > 0) && (position < 0)) {
		if ((desiredValue - position) >= 180){
			position = inertial.get_heading();
			turnV = 360 - desiredValue - abs(position);
		}
		else {
			turnV = (position + desiredValue);
		}
	}

	if (powerFunc == true){
	//   y =    a   +          bx            +           cx^2                  +                dx^3                +                fx^4
		kD = 0.769503+(0.279153*abs(desiredValue))+(-0.00232581*pow(abs(desiredValue), 2))+(0.0000156359*pow(abs(desiredValue), 3))+(-0.0000000398214*pow(abs(desiredValue), 4));
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
				turnV = (position + desiredValue);
			}
			else {
				turnV = (abs(position) + abs(desiredValue));
			}
		}
		else if ((desiredValue > 0) && (position < 0)) {
			if ((desiredValue - position) >= 180){
				position = inertial.get_heading();
				turnV = 360 - desiredValue - abs(position);
			}
			else {
				turnV = (position + desiredValue); //for different constants for pid
			}
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

	if (abs(error) <= 1000){
		kP = 0.75;
		kI = 0.000575; 
		kD = 3.3;
	}
	else if (abs(error) <= 4000){
		kP = 0.275;
		kI = 0.0007; //0.0007
		kD = 1.2489;
	}
	else {
		kP = 0.27;
		kI = 0.0007; //0.007
		kD = 1.248;
	}
	
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

	if (abs(error) <= 1000){
		kP2 = 0.75;
		kI2 = 0.000575; 
		kD2 = 3.3;
	}
	else if (abs(error) <= 4000){
		kP2 = 0.275;
		kI2 = 0.0007; //0.0007
		kD2 = 1.2489;
	}
	else {
		kP2 = 0.27;
		kI2 = 0.0007; //0.007
		kD2 = 1.248;
	}
	
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


void leftArc(double radius, double centralDegreeTheta, int timeout=1500, std::string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0){

	double rightArc = (centralDegreeTheta / 360)*2*M_PI*(radius + 530);
	double leftArc = (centralDegreeTheta / 360)*2*M_PI*(radius);

	bool chain = true;

	//double speedProp = rightArc/leftArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	bool taskStarted = false;
	bool taskEnded = false;

	double init_heading = inertial.get_heading(); 
	if (init_heading > 180){
		init_heading = init_heading - 360;
	}

	if (chainSpeed == 0){
		chain = false;
	}
	else {
		chain = true;
	}

	while(1){
		
		if (time > timeout){
			break;
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

		double leftcorrect = (currentLeftPosition * 360) / (2*M_PI*(radius)); 

		double heading = inertial.get_heading() - init_heading; 
		if(centralDegreeTheta > 0){ 
			if(heading > 30){
				heading = heading - 360; 
			}
		} else {
			if( heading > 300){ 
   				heading = heading - 360; 
			}
		}

		int fix = int(heading + leftcorrect);
		fix = fix*5;
		leftChassis.move(calculatePID(left_error) - fix);
		rightChassis.move(calculatePID2(right_error) + fix); //might need to add calcpid2

		if ((abs(leftArc - currentLeftPosition) <= 20) && (abs(rightArc - currentRightPosition) <= 20)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs(calculatePID(left_error)) <= chainSpeed && abs(calculatePID(right_error)) <= chainSpeed){ //might need to add calcpid2 back ON THE RIGHT SIDE
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

void rightArc(double radius, double centralDegreeTheta, int timeout=1500, std::string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0){


	double rightArc = (centralDegreeTheta / 360)*2*M_PI*(radius);
	double leftArc = (centralDegreeTheta / 360)*2*M_PI*(radius + 530);

	bool chain;

	//double speedProp = leftArc/rightArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	
	double init_heading = inertial.get_heading(); 
	if (init_heading > 180){
		init_heading = init_heading - 360;
	}
	bool taskStarted = false;
	bool taskEnded = false;
	
	// con.clear();

	if (chainSpeed == 0){
		chain = false;
	}
	else{
		chain = true;
	}

	while(1){

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

		double heading = inertial.get_heading() - init_heading; 
		if(centralDegreeTheta > 0){ 
			if(heading > 300){
				heading = heading - 360; 
			}
		} else {
			if( heading > 30){ 
   				heading = heading - 360; 
			}
		}

		int fix = int(heading - rightcorrect);
		fix = fix*5;
		leftChassis.move(calculatePID(left_error) - fix);
		rightChassis.move(calculatePID2(right_error) + fix); //might need to add calcpid2
		

		//con.print(0,0, "rc: %f", float(rightArc));

		if ((abs(leftArc - currentLeftPosition) <= 20) && (abs(rightArc - currentRightPosition) <= 20)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs(calculatePID(left_error)) <= chainSpeed && abs(calculatePID2(right_error)) <= chainSpeed){ //might need to add calcpid2 back ON THE RIGHT SIDE
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


void program1()
{
	//red rush
	drivePID(-1500, 1500);
	turnPID(-31);
	drivePID(-600);
	mogo.set_value(true);
	// delay(50);
	// drivePIDMogo(800,1500,10);
	// turnPIDMogo(0);
	// drivePIDMogo(500);
	// intake.move(127);
	// delay(750);
	// mogo.set_value(false);
	// turnPIDMogo(120);
	// intake.move(127);
	// drivePID(700);
	// intake.move(0);
	// turnPID(-90);
	// drivePID(-700);
	// mogo.set_value(true);
	// delay(50);
	// intake.move(127);
	// delay(1500);
}

void program2(){
	//blue rush
	drivePID(-1300, 1500, 5);
	turnPID(31);
	drivePID(-400);
	mogo.set_value(true);
	delay(50);
	drivePIDMogo(900,1500,10);
	turnPIDMogo(0);
	drivePIDMogo(500);
	intake.move(127);
	delay(750);
	turnPIDMogo(-120);
	mogo.set_value(false);
	intake.move(127);
	drivePID(800);
	intake.move(0);
	turnPID(90);
	drivePID(-700);
	mogo.set_value(true);
	delay(50);
	intake.move(127);
}

void program3()
{
	//blue ring side
	drivePID(-600);
	turnPID(31);
	drivePID(-550);
	mogo.set_value(true);
	delay(50);
	drivePID(-200);
	intake.move(127);
	delay(500);
	turnPIDMogo(-88);
	drivePID(900);
	turnPID(-170);
	drivePIDMogo(400);
	drivePIDMogo(-400);
	turnPIDMogo(-150);
	drivePIDMogo(800);
	drivePIDMogo(-1500, 2000);
	turnPID(-45);
	drivePID(-1300);
}

void skipAutonomous()
{
	
	//red rush
	drivePID(-1500, 1500);
	turnPID(-27);
	drivePID(-600, 1500, 0, true);
	delay(50);
	drivePIDMogo(800,1500,10);
	turnPIDMogo(0);
	drivePIDMogo(500);
	intake.move(127);
	// delay(750);
	// mogo.set_value(false);
	// turnPIDMogo(120);
	// intake.move(127);
	// drivePID(700);
	// intake.move(0);
	// turnPID(-90);
	// drivePID(-700);
	// mogo.set_value(true);
	// delay(50);
	// intake.move(127);
	// delay(1500);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	// //skills auto
	// intake.move(127);
	// delay(500);
	// drivePID(600);
	// turnPID(90);
	// drivePID(-500);
	// driveSPID(-300);
	// mogo.set_value(true);
	// delay(700);
	// turnPIDMogo(-27, 1500, false);
	// drivePIDMogo(900);
	// turnPIDMogo(-45);
	// drivePIDMogo(1800);
	// delay(300);
	// drivePIDMogo(-300);
	// turnPIDMogo(180);
	// delay(100);
	// drivePIDMogo(500);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	//red ring side
	// drivePID(-600);
	// turnPID(-31);
	// drivePID(-550);
	// mogo.set_value(true);
	// delay(50);
	// drivePID(-200);
	// intake.move(127);
	// delay(500);
	// turnPIDMogo(88);
	// drivePID(900);
	// turnPID(170);
	// drivePIDMogo(600);
	// drivePIDMogo(-600);
	// turnPIDMogo(160);
	// drivePIDMogo(600);
	// drivePID(-1250);
	// delay(100);
	// turnPID(45);
	// drivePID(-1800);

}
