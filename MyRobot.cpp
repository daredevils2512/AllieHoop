#include <cmath>
#include "WPILib.h"

/*
 * First iteration of Allie Hoop's program in C++
 */ 

	
	
class RobotDemo : public SimpleRobot
{

	static const int fender = 1250;
	static const int key = 2050;
	int autostate;
	int ballsInHigh;
	int ballsInLow;
	float xOutput;
	float yOutput;
	float twistOutput;
	float desiredFlywheelSpeed;
	float pidOutput;
	float scoopMotorValue;
	bool launcherOutSet;
	bool launcherInSet;
	bool waitForLeaving;
	bool fireButton;
	bool flywheelsOn;
	bool wheelsReady;
	bool autoscoop;
	bool runBrush;
	bool ballInTop;
	bool previousLowLightSensorValue;
	bool button1;

	Joystick stick1;
	Joystick stick2;
	Jaguar scoopMotor;
	Jaguar rearLeftMotor;
	Jaguar rearRightMotor;//inverted
	Jaguar frontLeftMotor;
	Jaguar frontRightMotor;//inverted
	Relay brushMotor;
	Relay elevator;
	Relay compressor;
	Victor bottomWheelsMotor;
	Solenoid launcherIn;
	Solenoid launcherOut;
	Solenoid wheelsDown;
	DigitalInput lowLightSensor;
	DigitalInput highLightSensor;
	DigitalInput scoopUp;
	DigitalInput scoopDown;
	DigitalInput compressorSwitch;
	Encoder leftWheels;
	Encoder rightWheels;
	Encoder bottomWheels;
	PIDController flywheelspeed;
	Timer stopwatch;
	Timer stopwatch1;
	AxisCamera& camera;	
	RobotDrive myRobot; // robot drive system

	

public:
	RobotDemo(void):	//these must be intialized in the same order
		autostate(0),
		ballsInHigh(0), 
		ballsInLow(0),
		xOutput(0),
		yOutput(0),
		twistOutput(0),
		desiredFlywheelSpeed(fender),
		pidOutput(0),
		scoopMotorValue(0),
		launcherOutSet(false),
		launcherInSet(false),
		waitForLeaving(true),
		fireButton(false),
		flywheelsOn(false),
		wheelsReady(false),
		autoscoop(false),
		runBrush(false),
		ballInTop(false),
		previousLowLightSensorValue(false),
		button1(false),
		stick1(1),		// as they are declared above.
		stick2(2),
		scoopMotor(1,5),
		rearLeftMotor(2,2),
		rearRightMotor(1,2),//inverted
		frontLeftMotor(2,1),
		frontRightMotor(1,1),//inverted
		brushMotor(2,2),
		elevator(2,1),
		compressor(1,4),
		bottomWheelsMotor(1,8),
		launcherIn(1,3),
		launcherOut(1,2),
		wheelsDown(1,1),
		lowLightSensor(2,7),
		highLightSensor(2,8),
		scoopUp(1,3),
		scoopDown(1,4),
		compressorSwitch(2,9),
		leftWheels(2,1,2,2),
		rightWheels(1,1,1,2),
		bottomWheels(2,5,2,6),
		flywheelspeed(3,.005,.0001,&bottomWheels,&bottomWheelsMotor),
		stopwatch(),
		stopwatch1(),
		camera(AxisCamera::GetInstance("192.168.0.2")),
		myRobot(&frontLeftMotor, &rearLeftMotor, &frontRightMotor, &rearRightMotor)
		//robotdrive function my accept speedcontoller memory locations as placeholders
		//for motors themselves
	{
		GetWatchdog().SetExpiration(0.1);
		flywheelspeed.SetOutputRange(0,3000);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	}



	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		wheelsDown.Set(true);
		leftWheels.Reset();
		rightWheels.Reset();
		while(!IsOperatorControl()){
			while(true){		//must run for whole autonomous
				bottomWheelsMotor.Set((flywheelspeed.Get())/3000);
				if(flywheelspeed.Get()<= 2550 && flywheelspeed.Get() >= 2200){
					wheelsReady = true;
				} else {
					wheelsReady = false;
				}
			}
			while(autostate == 0){//shoots first ball
				if(wheelsReady == true){
					Wait(.5);
					launcherIn.Set(true);
					launcherOut.Set(false);
					Wait(.25);
					launcherOut.Set(true);
					launcherIn.Set(false);
					wheelsReady = false;
					autostate = 1;
				}
			}
			while(autostate == 1){//moves second ball into position
				elevator.Set(Relay::kOn);
				Wait(2);
				elevator.Set(Relay::kOff);
				autostate = 2;
			}
			while(autostate == 2){//shoots second ball
				if(wheelsReady == true){
					Wait(.5);
					launcherIn.Set(true);
					launcherOut.Set(false);
					Wait(.25);
					launcherOut.Set(true);
					launcherIn.Set(false);
					wheelsReady = false;
					autostate = 3;
				}
			}
			while(autostate == 3 && stick2.GetThrottle() > 2){	//Drive to bridge
				if(leftWheels.Get() >= 3690 && rightWheels.Get() >= 3690){
					autostate = 4;
				}
				myRobot.TankDrive(.5, (leftWheels.Get() - rightWheels.Get())*.001 + .5);
			}
			stopwatch.Reset();
			stopwatch.Start();
			while (autostate == 4 && stick2.GetThrottle() > 2){ //Tip bridge
				if(stopwatch.Get() >= 3){
					autostate = 5;
				}
				if(!scoopDown.Get()){
					scoopMotor.Set(1);
				}
				else{
					scoopMotor.Set(0);
					autostate = 5;
				}
			}
			stopwatch.Stop();
			stopwatch.Reset();
			while(autostate == 5 && stick2.GetThrottle() > 2){
				Wait(3);
				autostate = 6;
			}
			stopwatch.Start();
			while(autostate == 6 && stick2.GetThrottle() > 2){
				
				if(stopwatch.Get() >= 2 ){
					scoopMotor.Set(0);
					autostate = 7;
				}
				if(!scoopUp.Get()){
					scoopMotor.Set(-.6);
				}
				else{
					scoopMotor.Set(0);
					autostate = 7;
				}
			}
			stopwatch.Stop();
			stopwatch.Reset();
		}
	}

	void OperatorControl(void)
	{
		wheelsDown.Set(false);
		GetWatchdog().SetEnabled(false);
		while (IsOperatorControl())
		{
			//Sensitivity of Joystick
			//X
			if(stick1.GetX() >= .05){
				xOutput = pow((stick1.GetX()*.05), 2);
			}
			else if(stick1.GetX() <= -.05){
				xOutput = pow((stick1.GetX()*.05), 2)*-1;
			}
			else{
				xOutput = 0;
			}
			//Y
			if(stick1.GetY() >= .05){
				yOutput = pow((stick1.GetY()*.05), 2);
			}
			else if(stick1.GetY() <= -.05){
				yOutput = pow((stick1.GetY()*.05), 2)*-1;
			}
			else{
				yOutput = 0;
			}
			//Twist
			if(stick1.GetTwist() >= .05){
				twistOutput = pow((stick1.GetTwist()*.05), 2);
			}
			else if(stick1.GetX() <= -.05){
				twistOutput = pow((stick1.GetTwist()*.05), 2)*-1;
			}
			else{
				twistOutput = 0;
			}
			if(stick1.GetThrottle() >= 0){
				twistOutput = twistOutput/2;
			}
			else{
				twistOutput = twistOutput/1.5;
			}
			//Grippy Deployment
			if(stick1.GetRawButton(2)){
				xOutput = 0;
				wheelsDown.Set(true);
			}
			else{
				wheelsDown.Set(false);
			}
			//Drive with Mecanum style
			myRobot.MecanumDrive_Cartesian(xOutput, yOutput, twistOutput);
			//checks for current scoop mode
			if(stick1.GetRawButton(12) || stick1.GetRawButton(11)){
				autoscoop = true;
			}
			else if(stick1.GetRawButton(7) || stick1.GetRawButton(9)){
				autoscoop = false;
			}
			if(stick1.GetRawButton(2)){
				wheelsDown.Set(true);
			}
			else{
				wheelsDown.Set(false);
			}
			//Autoscoop
			if(autoscoop == true){
				if(scoopUp.Get()){
					runBrush = false;
				}
				else if(stick1.GetRawButton(7)){
					runBrush = true;
				}
				if(stick1.GetRawButton(7) || stick1.GetRawButton(9)){
					if(scoopDown.Get()){
						scoopMotorValue = 0;
					}
					else if(stick1.GetRawButton(9)){
						scoopMotorValue = 1;
					}
					else{
						scoopMotorValue = .4;
					}
				}
				else if(scoopUp.Get()){
					scoopMotorValue = 0;
				}
				else{
					scoopMotorValue = -.85;
				}
			}
			else{
				//Manual scoop
				if(stick1.GetRawButton(11)){
					if(scoopDown.Get()){
					scoopMotorValue = 0;
					}
					else{
						scoopMotorValue = .6;
					}
				}
				else if(stick1.GetRawButton(12)){
					if(scoopUp.Get()){
						scoopMotorValue = 0;
					}
					else{
						scoopMotorValue = -.6;
					}
				}
				else{
					scoopMotorValue = 0;	
				}
				if(stick2.GetRawButton(8)){
					runBrush = true;
				}
				else{
					runBrush = false;
				}
			}
			//Ball Position
			if(!previousLowLightSensorValue && lowLightSensor.Get()){
				ballsInLow++;
				previousLowLightSensorValue = true ;
			}
			if (previousLowLightSensorValue && !lowLightSensor.Get()){
				previousLowLightSensorValue = false;
			}
			if(highLightSensor.Get()){
				stopwatch1.Reset();
				stopwatch1.Start();
			}
			if(stopwatch1.Get() >= 2.5){
				ballsInHigh++;
				ballsInLow--;
				stopwatch1.Stop();
				stopwatch1.Reset();
			}
			if(ballsInHigh > 0){
				ballInTop = true;
			}
			else{
				ballInTop = false;
			}
			//Elevator
			if(stick2.GetRawButton(10) || stick2.GetRawButton(11)){
				if(stick2.GetRawButton(10)){
					elevator.Set(Relay::kReverse);
				}
				else if(stick2.GetRawButton(11)){
					elevator.Set(Relay::kForward);
				}
				else{
					elevator.Set(Relay::kOff);
				}
			}
			else if(ballsInLow > 0 || (ballsInHigh == 1 && ballsInLow > 0)){
				elevator.Set(Relay::kForward);
			}
			else{
				elevator.Set(Relay::kOff);
			}
			//Flywheel speed
			if(stick2.GetRawButton(3)){  
				desiredFlywheelSpeed = key;
			}
			else if(stick2.GetRawButton(4)){
				desiredFlywheelSpeed = (desiredFlywheelSpeed - 100);
			}
			else if(stick2.GetRawButton(5)){
				desiredFlywheelSpeed = (desiredFlywheelSpeed + 100);
			}
			flywheelspeed.SetSetpoint(desiredFlywheelSpeed);
			if(stick2.GetRawButton(9)){
				flywheelsOn = true;
			}
			else if(stick2.GetRawButton(6)){
				flywheelsOn = false;
			}
			if(flywheelsOn){
				bottomWheelsMotor.Set((flywheelspeed.Get())/3000);
			}
			else{
				bottomWheelsMotor.Set(0);
			}
			//Actuators
			if(stick2.GetRawButton(1) && !fireButton){
				stopwatch.Reset();
				stopwatch.Start();
				waitForLeaving = false;
				button1 = true;
			}
			if(stopwatch.Get() >= .25 || waitForLeaving == true){
				if(stopwatch.Get() >= .5 || waitForLeaving == true){
					launcherOutSet = false;
					launcherInSet = false;
				}
				else{
					launcherOutSet = true;
					launcherInSet = false;
				}
			}
			else{
				launcherOutSet = false;
				launcherInSet = true;
			}
			launcherIn.Set(launcherInSet);
			launcherOut.Set(launcherOutSet);
			if(stopwatch.Get() >= 5){
				stopwatch.Stop();
			}
			//Makes 1 time button work
			if(button1){
				fireButton = true;
				button1 = false;
			}
			else{
				fireButton = false;
			}
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

