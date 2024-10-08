#include "main.h"
#include "global.h"

using namespace pros;
using namespace glb;
using namespace std;


void drivePID(int desiredValue, int timeout=1500)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;

	double kP = 0.75;
	double kI = 0.000575; 
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

		double currentIMUValue = inertial.get_heading();
		if (currentIMUValue > 180){
			currentIMUValue = currentIMUValue - 360;
		}
		double headingCorrection = initialValue -currentIMUValue;
		headingCorrection = headingCorrection * 5;



		// get avg of motors:
		int currentValue = (FRpos + BRpos + FLpos + BLpos) / 4;

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
			totalError = min(totalError, maxI);
		}
		else{
			totalError = max(totalError, -maxI);
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

		delay(20);

		time++; //add one to time every cycle
		
	}

	chassis.move(0);
}

void turnPID(int desiredValue, int timeout=1500)
{
	bool enableTurnPID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	double position;
	double turnV;

	double kP = 8;
	double kI = 0.0001; 
	double kD = 30;
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
		}
		else {
			turnV = (position + desiredValue);
		}
	}

	// if (abs(turnV)<10){
	// 	//constants
	// }
	// else if (abs(turnV < 20)){
	// 	//constants
	// }
	// else if (abs(turnV < 30)){
	// 	//constants
	// }
	// else if (abs(turnV < 40)){
	// 	//constants
	// }
	// else if (abs(turnV < 50)){
	// 	//constants
	// }
	// else if (abs(turnV < 70)){
	// 	//constants
	// }
	// else if (abs(turnV <= 50)){
	// 	kP = 8;
	// 	kI = 0; 
	// 	kD = 0;
	// }
	// else if (abs(turnV <= 90)){
	// 	kP = 8;
	// 	kI = 0.0001; 
	// 	kD = 30;
	// }

	//tune distances
	//test intervals


	while (enableTurnPID)
	{
		if (time > timeout){
			enableTurnPID = false;
		}

		// get avg of motors:
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
			totalError = min(totalError, maxI);
		}
		else{
			totalError = max(totalError, -maxI);
		}

		con.print(0,0, "error: %f", float(error));

		double speed = (error * kP + derivative * kD + totalError * kI);
		rightChassis.move(-speed);
		leftChassis.move(speed); //might need to flip these in case i got the wrong lines

		prevError = error;

		if (error < 5)
		{
			count++;
		}

		if (count > 30)
		{
			enableTurnPID = false;
		}

		delay(20);
		time++;
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


void leftArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0){

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
		leftChassis.move(calculatePID(left_error) + fix);
		rightChassis.move(calculatePID(right_error) - fix); //might need to add calcpid2

		if ((abs(leftArc - currentLeftPosition) <= 20) && (abs(rightArc - currentRightPosition) <= 20)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs(calculatePID(left_error)) <= chainSpeed && abs(calculatePID2(right_error)) <= chainSpeed){
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

void rightArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0){


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
		leftChassis.move(calculatePID(left_error) + fix);
		rightChassis.move(calculatePID(right_error) - fix); //might need to add calcpid2
		

		//con.print(0,0, "rc: %f", float(rightArc));

		if ((abs(leftArc - currentLeftPosition) <= 20) && (abs(rightArc - currentRightPosition) <= 20)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs(calculatePID(left_error)) <= chainSpeed && abs(calculatePID2(right_error)) <= chainSpeed){
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
