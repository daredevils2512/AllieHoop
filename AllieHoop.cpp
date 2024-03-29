#include <cmath>
#include <iostream>
#include "AllieHoop.h"

/*
 * First iteration of Allie Hoop's program in C++
 */ 


const float RobotDemo::fender = 1250;
const float RobotDemo::key = 2050;


RobotDemo::RobotDemo(void):	//these must be intialized in the same order
	autostate(0),
	ballsInHigh(0), 
	ballsInLow(0),
	pidOutput(0),
	scoopMotorValue(0),
	launcherOutSet(false),
	launcherInSet(false),
	waitForLeaving(true),
	fireButton(false),
	flywheelsOn(true),
	autoscoop(false),
	runBrush(false),
	ballInTop(false),
	previousLowLightSensorValue(false),
	button1(false),
	stick1(1),		// as they are declared above.
	stick2(2),
	scoopMotor(SCOOP_MOTOR_SIDECAR, SCOOP_MOTOR_PWM),
	rearLeftMotor(REAR_LEFT_MOTOR_SIDECAR, REAR_LEFT_MOTOR_PWM),
	rearRightMotor(REAR_RIGHT_MOTOR_SIDECAR, REAR_RIGHT_MOTOR_PWM),//inverted
	frontLeftMotor(FRONT_LEFT_MOTOR_SIDECAR, FRONT_LEFT_MOTOR_PWM),
	frontRightMotor(FRONT_RIGHT_MOTOR_SIDECAR, FRONT_RIGHT_MOTOR_PWM),//inverted
	brushMotor(BRUSH_MOTOR_SIDECAR, BRUSH_MOTOR_PWM),
	elevator(ELEVATOR_SIDECAR, ELEVATOR_PWM),
	compressor(COMPRESSOR_SIDECAR, COMPRESSOR_PWM),
	bottomWheelsMotor(BOTTOM_WHEELS_MOTOR_SIDECAR, BOTTOM_WHEELS_MOTOR_PWM),
	launcherIn(LAUNCHER_IN_SIDECAR, LAUNCHER_IN_PWM),
	launcherOut(LAUNCHER_OUT_SIDECAR, LAUNCHER_OUT_PWM),
	wheelsDown(WHEELS_DOWN_SIDECAR, WHEELS_DOWN_PWM),
	lowLightSensor(LOW_LIGHT_SENSOR_SIDECAR, LOW_LIGHT_SENSOR_PWM),
	highLightSensor(HIGH_LIGHT_SENSOR_SIDECAR, HIGH_LIGHT_SENSOR_PWM),
	scoopUp(SCOOP_UP_SIDECAR, SCOOP_UP_PWM),
	scoopDown(SCOOP_DOWN_SIDECAR, SCOOP_DOWN_PWM),
	compressorSwitch(COMPRESSOR_SWITCH_SIDECAR, COMPRESSOR_SWITCH_PWM),
	leftWheels(REAR_LEFT_MOTOR_SIDECAR, REAR_LEFT_MOTOR_PWM, FRONT_LEFT_MOTOR_SIDECAR, FRONT_LEFT_MOTOR_PWM),
	rightWheels(REAR_RIGHT_MOTOR_SIDECAR, REAR_RIGHT_MOTOR_PWM, FRONT_RIGHT_MOTOR_SIDECAR, FRONT_RIGHT_MOTOR_PWM),
	bottomWheels(BOTTOM_WHEELS_ENCODER_SIDECAR_A, BOTTOM_WHEELS_ENCODER_PWM_A, BOTTOM_WHEELS_ENCODER_SIDECAR_B, BOTTOM_WHEELS_ENCODER_PWM_B),
	flywheelspeed(3, 0.005, 0.0001, &bottomWheels, &bottomWheelsMotor),
	stopwatch(),
	stopwatch1(),
	myRobot(&frontLeftMotor, &rearLeftMotor, &frontRightMotor, &rearRightMotor)
	//robotdrive function my accept speedcontoller memory locations as placeholders
	//for motors themselves
{
	GetWatchdog().SetExpiration(10);
	stick1.SetAxisChannel(Joystick::kTwistAxis, 3);
	stick1.SetAxisChannel(Joystick::kThrottleAxis, 4);
	flywheelspeed.SetOutputRange(0, 3000);
	myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
}



void RobotDemo::Autonomous(void)
{
	cout << "Beginning Autonomous\n";
	GetWatchdog().SetEnabled(false);
	wheelsDown.Set(true);
	leftWheels.Reset();
	rightWheels.Reset();

	while (autostate == 0 && IsAutonomous()) {
		float currentFlywheelSpeed = flywheelspeed.Get();
		bottomWheelsMotor.Set(currentFlywheelSpeed / 3000);
		if (currentFlywheelSpeed <= 2550 && currentFlywheelSpeed >= 2200) {
			autostate = 1;
		}
	}
	if (IsAutonomous()) {
		cout << "Begin..Shoot first ball\n";
		//Begin...Shoot First Ball
		launcherIn.Set(true);
		launcherOut.Set(false);
		Wait(0.25);
		launcherOut.Set(true);
		launcherIn.Set(false);
		//End...Shoot First Ball
		
		//Begin...Run Elevator
		elevator.Set(Relay::kOn);
		Wait(2);
		elevator.Set(Relay::kOff);
		//End...Run Elevator
		while (autostate == 1 && IsAutonomous()) {
			float currentFlywheelSpeed = flywheelspeed.Get();
			bottomWheelsMotor.Set(currentFlywheelSpeed / 3000);
			if (currentFlywheelSpeed <= 2550 && currentFlywheelSpeed >= 2200) {
				autostate = 2;
			}
		}
	}
	if (IsAutonomous()) {
		cout << "Shoot second ball\n";
		//Begin...Shoot Second Ball
		launcherIn.Set(true);
		launcherOut.Set(false);
		Wait(0.25);
		launcherOut.Set(true);
		launcherIn.Set(false);
	}
	//Begin...Drive to bridge
	if (stick2.GetThrottle() > 2) {
		cout << "begin driving to bridge\n";
		while (autostate == 2 && IsAutonomous()) {
			if (leftWheels.Get() >= 3690 && rightWheels.Get() >= 3690) {
				autostate = 3;
			}
			myRobot.TankDrive(0.5, (leftWheels.Get() - rightWheels.Get())*0.001 + 0.5);
			cout << "Drive to bridge stopped\n";
		}//End...Drive to Bridge
	}
	stopwatch.Reset();
	stopwatch.Start();
	 //Tip bridge
	if (stick2.GetThrottle() > 2) {
		cout << "Tipping bridge\n";
		while (autostate == 3 && IsAutonomous()) {
			if (stopwatch.Get() >= 3 && IsAutonomous()) {
				autostate = 4;
			}
			if (!scoopDown.Get() && IsAutonomous()) {
				scoopMotor.Set(1);
			}
			else {
				scoopMotor.Set(0);
				autostate = 4;
			}
		}
	}
	stopwatch.Stop();
	stopwatch.Reset();
	if (stick2.GetThrottle() > 2 && IsAutonomous()) {
		Wait(3);
	}
	stopwatch.Start();
	if (stick2.GetThrottle() > 2 && IsAutonomous()) {
		while (autostate == 4 && IsAutonomous()) {
			if (stopwatch.Get() >= 2 ) {
				scoopMotor.Set(0);
				autostate = 5;
			}
			if (!scoopUp.Get()) {
				scoopMotor.Set(-0.6);
			}
			else {
				scoopMotor.Set(0);
				autostate = 5;
			}
		}
	}
	stopwatch.Stop();
	stopwatch.Reset();
	cout << "Tipping bridge ended\n";
}//End...Tip bridge


void RobotDemo::OperatorControl(void)
{
	wheelsDown.Set(false);
	GetWatchdog().SetEnabled(false);
	while (1 == 1)
	{
		Drive();
		Scoop();
		Elevator();
		Shoot();
	}
}

void RobotDemo::Drive()
{
	//Sensitivity of Joystick
	//X
	float xInput = stick1.GetX();
	float xOutput = ConvertAxis(xInput);
	//Y
	float yInput = stick1.GetY();
	float yOutput = ConvertAxis(yInput);
	//Twist
	float twistInput = stick1.GetTwist();
	float twistOutput = ConvertAxis(twistInput * 1);//Twist is inverted
		
	if (stick1.GetThrottle() >= 0) {
		twistOutput = twistOutput/2;
	}
	else {
		twistOutput = twistOutput/1.5;
	}
	//Grippy Deployment
	if (stick1.GetRawButton(GRIPPYS_DOWN)) {
		xOutput = 0;
		wheelsDown.Set(true);
	}
	else {
		wheelsDown.Set(false);
	}
	//Drive with Mecanum style
	myRobot.MecanumDrive_Cartesian(xOutput, yOutput, twistOutput);
}

float RobotDemo::ConvertAxis(float input){
	if (input >= 0.05) {
		return pow((input*0.75f), 2);
	}
	else if (input <= -0.05) {
		return pow((input*0.75f), 2)*-1;
	}
	else {
		return 0;
	}
}

void RobotDemo::Scoop()
{
	bool scoopBallsButton = stick1.GetRawButton(SCOOP_BALLS);
	bool scoopBridgeButton = stick1.GetRawButton(SCOOP_BRIDGE);
	bool upperLimitSwitch = scoopUp.Get();
	bool lowerLimitSwitch = scoopDown.Get();
	//checks for current scoop mode
	if (scoopBallsButton || scoopBridgeButton) {
		autoscoop = true;
	}
	else if (stick1.GetRawButton(SCOOP_DOWN) || stick1.GetRawButton(SCOOP_UP)) {
		autoscoop = false;
	}
	//Autoscoop
	


	if (autoscoop == true) {
		if (upperLimitSwitch) {
			brushMotor.Set(Relay::kOff);
		}
		else if (scoopBallsButton) {
			brushMotor.Set(Relay::kForward);
		}
		if (scoopBallsButton || scoopBridgeButton) {
			if (!lowerLimitSwitch) {
				scoopMotor.Set(0);
			}
			else if (scoopBridgeButton) {
				scoopMotor.Set(1);
			}
			else if (scoopBallsButton) {
					scoopMotor.Set(0.4f);
			}
		}
		else if (upperLimitSwitch) {
			scoopMotor.Set(0);
		}
		else {
				scoopMotor.Set(-0.85f);
		}
	}
	else {
		//Manual scoop
		if (stick1.GetRawButton(SCOOP_DOWN)) {
			if (!lowerLimitSwitch) {
				scoopMotor.Set(0);
			}
			else {
					scoopMotor.Set(0.6f);
			}
		}
		else if (stick1.GetRawButton(SCOOP_UP)) {
			if (upperLimitSwitch) {
				scoopMotor.Set(0);
			}
			else {
					scoopMotor.Set(-0.6f);
			}
		}
		else{
			scoopMotor.Set(0);
		}
		if (stick2.GetRawButton(RUN_BRUSH_MOTOR)) {
			brushMotor.Set(Relay::kForward);
		}
		else {
			brushMotor.Set(Relay::kOff);
		}
	}
}

void RobotDemo::Elevator()
{
	static bool previousHighLightSensorValue = false;
	//Ball Position
	if (previousLowLightSensorValue == false && lowLightSensor.Get()) {
		ballsInLow++;
		previousLowLightSensorValue = true ;
	}
	else if (previousLowLightSensorValue && lowLightSensor.Get() == false) {
		previousLowLightSensorValue = false;
	}
	if (highLightSensor.Get()) {
		if(ballInTop && !previousHighLightSensorValue){
			ballsInHigh++;
			ballsInLow--;
			previousHighLightSensorValue = true;
		}
		else if(!previousHighLightSensorValue){
			stopwatch1.Reset();
			stopwatch1.Start();	
		}
	}
	if(previousHighLightSensorValue && !highLightSensor.Get()){
		previousHighLightSensorValue = false;
	}
	if (stopwatch1.Get() >= 2.5) {
			ballsInHigh++;
			ballsInLow--;
			stopwatch1.Stop();
			stopwatch1.Reset();
	}
	if (ballsInHigh > 0) {
		ballInTop = true;
	}
	else {
		ballInTop = false;
	}
	//Elevator
	if (stick2.GetRawButton(ELEVATOR_REVERSE)) {
			elevator.Set(Relay::kReverse);
		}
		else if (stick2.GetRawButton(ELEVATOR_FOWARD)) {
			elevator.Set(Relay::kForward);
			}
	else if (ballsInLow > 0 && ballsInHigh < 2) {
		elevator.Set(Relay::kForward);
	}
	else {
		elevator.Set(Relay::kOff);
	}
}



void RobotDemo::Shoot()
{
	//Flywheel speed
	float currentSpeed = 0;
	float desiredFlywheelSpeed = fender;
	
	if (stick2.GetRawButton(KEY_BUTTON)) {  
		desiredFlywheelSpeed = key;
	}
	else if (stick2.GetRawButton(FENDER_BUTTON)){
		desiredFlywheelSpeed = fender;
	}
	else if (stick2.GetRawButton(REDUCE_FLYWHEEL_SPEED)) {
		desiredFlywheelSpeed = (desiredFlywheelSpeed - 100);
	}
	else if (stick2.GetRawButton(INCREASE_FLYWHEEL_SPEED)) {
		desiredFlywheelSpeed = (desiredFlywheelSpeed + 100);
	}
	flywheelspeed.SetSetpoint(desiredFlywheelSpeed);
	if (stick2.GetRawButton(FLYWHEELS_ON)) {
		flywheelsOn = true;
	}
	else if (stick2.GetRawButton(FLYWHEELS_OFF)) {
		flywheelsOn = false;
	}
	if (flywheelsOn) {
			bottomWheelsMotor.Set(flywheelspeed.Get() / 3000);
			currentSpeed = (flywheelspeed.Get() / 3000);
			printf("PID = %f\n", currentSpeed);
	}
	else {
		bottomWheelsMotor.Set(0);
	}
	//Actuators
	if (stick2.GetRawButton(FIRE_BUTTON) && !fireButton) {
		stopwatch.Reset();
		stopwatch.Start();
		waitForLeaving = false;
		button1 = true;
	}
	if (stopwatch.Get() >= 0.25 || waitForLeaving == true) {
		if (stopwatch.Get() >= 0.5 || waitForLeaving == true) {
			launcherOutSet = false;
			launcherInSet = false;
		}
		else {
			launcherOutSet = true;
			launcherInSet = false;
		}
	}
	else {
		launcherOutSet = false;
		launcherInSet = true;
	}
	launcherIn.Set(launcherInSet);
	launcherOut.Set(launcherOutSet);
	if (stopwatch.Get() >= 5) {
		stopwatch.Stop();
	}
	//Makes 1 time button work
	if (button1) {
		fireButton = true;
		button1 = false;
	}
	else {
		fireButton = false;
	}
}

START_ROBOT_CLASS(RobotDemo);
