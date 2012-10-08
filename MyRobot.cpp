#include <cmath>
#include "WPILib.h"

/*
 * First iteration of Allie Hoop's program in C++
 */ 



class RobotDemo : public SimpleRobot
{
	static const UINT8 ANALOG_SIDECAR_MODULE_1 = 1;
	static const UINT8 DIGITAL_SIDECAR_MODULE_1 = 2;
	static const UINT8 SOLENOID_SIDECAR_MODULE_1 = 3;
	static const UINT8 ANALOG_SIDECAR_MODULE_2 = 5;
	static const UINT8 DIGITAL_SIDECAR_MODULE_2 = 6;
	static const UINT8 SOLENOID_SIDECAR_MODULE_2 = 7;
	
	// Sidecars
	static const UINT8 SCOOP_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 REAR_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 REAR_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 FRONT_LEFT_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 FRONT_RIGHT_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 BRUSH_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 ELEVATOR_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 COMPRESSOR_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 BOTTOM_WHEELS_MOTOR_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 LAUNCHER_IN_SIDECAR = SOLENOID_SIDECAR_MODULE_1;
	static const UINT8 LAUNCHER_OUT_SIDECAR = SOLENOID_SIDECAR_MODULE_1;
	static const UINT8 WHEELS_DOWN_SIDECAR = SOLENOID_SIDECAR_MODULE_1;
	static const UINT8 LOW_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 HIGH_LIGHT_SENSOR_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 SCOOP_UP_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 SCOOP_DOWN_SIDECAR = DIGITAL_SIDECAR_MODULE_1;
	static const UINT8 COMPRESSOR_SWITCH_SIDECAR = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 BOTTOM_WHEELS_ENCODER_SIDECAR_A = DIGITAL_SIDECAR_MODULE_2;
	static const UINT8 BOTTOM_WHEELS_ENCODER_SIDECAR_B = DIGITAL_SIDECAR_MODULE_2;

	// PWMS
	static const UINT32 SCOOP_MOTOR_PWM = 5;
	static const UINT32 REAR_LEFT_MOTOR_PWM = 2;
	static const UINT32 REAR_RIGHT_MOTOR_PWM = 2;
	static const UINT32 FRONT_LEFT_MOTOR_PWM = 1;
	static const UINT32 FRONT_RIGHT_MOTOR_PWM = 1;
	static const UINT32 BRUSH_MOTOR_PWM = 2;
	static const UINT32 ELEVATOR_PWM = 1;
	static const UINT32 COMPRESSOR_PWM = 4;
	static const UINT32 BOTTOM_WHEELS_MOTOR_PWM = 8;
	static const UINT32 LAUNCHER_IN_PWM = 3;
	static const UINT32 LAUNCHER_OUT_PWM = 2;
	static const UINT32 WHEELS_DOWN_PWM = 1;
	static const UINT32 LOW_LIGHT_SENSOR_PWM = 7;
	static const UINT32 HIGH_LIGHT_SENSOR_PWM = 8;
	static const UINT32 SCOOP_UP_PWM = 3;		// LIMIT SWITCH
	static const UINT32 SCOOP_DOWN_PWM = 4;// LIMIT SWITCH
	static const UINT32 COMPRESSOR_SWITCH_PWM = 9;
	static const UINT32 BOTTOM_WHEELS_ENCODER_PWM_A = 5;
	static const UINT32 BOTTOM_WHEELS_ENCODER_PWM_B = 6;

	// Joystick 1 buttons
	static const UINT32 GRIPPYS_DOWN = 2;
	static const UINT32 SCOOP_BRIDGE = 9;
	static const UINT32 SCOOP_BALLS = 7;
	static const UINT32 SCOOP_DOWN = 11;
	static const UINT32 SCOOP_UP = 12;

	// Joystick 2 buttons
	static const UINT32 FIRE_BUTTON = 1;
	static const UINT32 ELEVATOR_FOWARD = 11;
	static const UINT32 ELEVATOR_REVERSE = 10;
	static const UINT32 KEY_BUTTON = 3;
	static const UINT32 REDUCE_FLYWHEEL_SPEED = 4;
	static const UINT32 INCREASE_FLYWHEEL_SPEED = 5;
	static const UINT32 FLYWHEELS_ON = 9;
	static const UINT32 FLYWHEELS_OFF = 6;
	static const UINT32 RUN_BRUSH_MOTOR = 8;

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
		autoscoop(false),
		runBrush(false),
		ballInTop(false),
		previousLowLightSensorValue(false),
		button1(false),
		stick1(1),		// as they are declared above.
		stick2(2),
		scoopMotor(SCOOP_MOTOR_SIDECAR,SCOOP_MOTOR_PWM),
		rearLeftMotor(REAR_LEFT_MOTOR_SIDECAR,REAR_LEFT_MOTOR_PWM),
		rearRightMotor(REAR_RIGHT_MOTOR_SIDECAR,REAR_RIGHT_MOTOR_PWM),//inverted
		frontLeftMotor(FRONT_LEFT_MOTOR_SIDECAR,FRONT_LEFT_MOTOR_PWM),
		frontRightMotor(FRONT_RIGHT_MOTOR_SIDECAR,FRONT_RIGHT_MOTOR_PWM),//inverted
		brushMotor(BRUSH_MOTOR_SIDECAR,BRUSH_MOTOR_PWM),
		elevator(ELEVATOR_SIDECAR,ELEVATOR_PWM),
		compressor(COMPRESSOR_SIDECAR,COMPRESSOR_PWM),
		bottomWheelsMotor(BOTTOM_WHEELS_MOTOR_SIDECAR,BOTTOM_WHEELS_MOTOR_PWM),
		launcherIn(LAUNCHER_IN_SIDECAR,LAUNCHER_IN_PWM),
		launcherOut(LAUNCHER_OUT_SIDECAR,LAUNCHER_OUT_PWM),
		wheelsDown(WHEELS_DOWN_SIDECAR,WHEELS_DOWN_PWM),
		lowLightSensor(LOW_LIGHT_SENSOR_SIDECAR,LOW_LIGHT_SENSOR_PWM),
		highLightSensor(HIGH_LIGHT_SENSOR_SIDECAR,HIGH_LIGHT_SENSOR_PWM),
		scoopUp(SCOOP_UP_SIDECAR,SCOOP_UP_PWM),
		scoopDown(SCOOP_DOWN_SIDECAR,SCOOP_DOWN_PWM),
		compressorSwitch(COMPRESSOR_SWITCH_SIDECAR,COMPRESSOR_SWITCH_PWM),
		leftWheels(REAR_LEFT_MOTOR_SIDECAR,REAR_LEFT_MOTOR_PWM,FRONT_LEFT_MOTOR_SIDECAR,FRONT_LEFT_MOTOR_PWM),
		rightWheels(REAR_RIGHT_MOTOR_SIDECAR,REAR_RIGHT_MOTOR_PWM,FRONT_RIGHT_MOTOR_SIDECAR,FRONT_RIGHT_MOTOR_PWM),
		bottomWheels(BOTTOM_WHEELS_ENCODER_SIDECAR_A,BOTTOM_WHEELS_ENCODER_PWM_A,BOTTOM_WHEELS_ENCODER_SIDECAR_B,BOTTOM_WHEELS_ENCODER_PWM_B),
		flywheelspeed(3,0.005,0.0001,&bottomWheels,&bottomWheelsMotor),
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

		while(autostate == 0 && IsAutonomous()){
			bottomWheelsMotor.Set((flywheelspeed.Get())/3000);
			if(flywheelspeed.Get()<= 2550 && flywheelspeed.Get() >= 2200){
				autostate = 1;
			}
		}
		if(IsAutonomous()){
			//Begin...Shoot First Ball
			launcherIn.Set(true);
			launcherOut.Set(false);
			Wait(0.25);
			launcherOut.Set(true);
			launcherIn.Set(false);
			//End...Shoot First Ball
			elevator.Set(Relay::kOn);//Begin...Run Elevator
			Wait(2);
			elevator.Set(Relay::kOff);//End...Run Elevator
			while(autostate == 1 && IsAutonomous()){
				bottomWheelsMotor.Set((flywheelspeed.Get())/3000);
				if(flywheelspeed.Get()<= 2550 && flywheelspeed.Get() >= 2200){
					autostate = 2;
				}
			}
		}
		if(IsAutonomous()){
			//Begin...Shoot Second Ball
			launcherIn.Set(true);
			launcherOut.Set(false);
			Wait(0.25);
			launcherOut.Set(true);
			launcherIn.Set(false);
		}
		if(stick2.GetThrottle() > 2){//Begin...Drive to bridge
			while(autostate == 2 && IsAutonomous()){
				if(leftWheels.Get() >= 3690 && rightWheels.Get() >= 3690){
					autostate = 3;
				}
				myRobot.TankDrive(0.5, (leftWheels.Get() - rightWheels.Get())*0.001 + 0.5);
			}//End...Drive to Bridge
		}
		stopwatch.Reset();
		stopwatch.Start();
		if (stick2.GetThrottle() > 2){ //Tip bridge
			while(autostate == 3 && IsAutonomous()){
				if(stopwatch.Get() >= 3 && IsAutonomous()){
					autostate = 4;
				}
				if(!scoopDown.Get() && IsAutonomous()){
					scoopMotor.Set(1);
				}
				else{
					scoopMotor.Set(0);
					autostate = 4;
				}
			}
		}
		stopwatch.Stop();
		stopwatch.Reset();
		if(stick2.GetThrottle() > 2 && IsAutonomous()){
			Wait(3);
		}
		stopwatch.Start();
		if(stick2.GetThrottle() > 2 && IsAutonomous()){
			while(autostate == 4 && IsAutonomous()){
				if(stopwatch.Get() >= 2 ){
					scoopMotor.Set(0);
					autostate = 5;
				}
				if(!scoopUp.Get()){
					scoopMotor.Set(-0.6);
				}
				else{
					scoopMotor.Set(0);
					autostate = 5;
				}
			}
		}
		stopwatch.Stop();
		stopwatch.Reset();
	}//End...Tip bridge


	void OperatorControl(void)
	{
		wheelsDown.Set(false);
		GetWatchdog().SetEnabled(false);
		while (IsOperatorControl())
		{
			//Sensitivity of Joystick
			//X
			if(stick1.GetX() >= 0.05){
				xOutput = pow((stick1.GetX()*0.05), 2);
			}
			else if(stick1.GetX() <= -0.05){
				xOutput = pow((stick1.GetX()*0.05), 2)*-1;
			}
			else{
				xOutput = 0;
			}
			//Y
			if(stick1.GetY() >= 0.05){
				yOutput = pow((stick1.GetY()*0.05), 2);
			}
			else if(stick1.GetY() <= -0.05){
				yOutput = pow((stick1.GetY()*0.05), 2)*-1;
			}
			else{
				yOutput = 0;
			}
			//Twist
			if(stick1.GetTwist() >= 0.05){
				twistOutput = pow((stick1.GetTwist()*0.05), 2);
			}
			else if(stick1.GetTwist() <= -0.05){
				twistOutput = pow((stick1.GetTwist()*0.05), 2)*-1;
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
			if(stick1.GetRawButton(GRIPPYS_DOWN)){
				xOutput = 0;
				wheelsDown.Set(true);
			}
			else{
				wheelsDown.Set(false);
			}
			//Drive with Mecanum style
			myRobot.MecanumDrive_Cartesian(xOutput, yOutput, twistOutput);
			//checks for current scoop mode
			if(stick1.GetRawButton(SCOOP_BALLS) || stick1.GetRawButton(SCOOP_BRIDGE)){
				autoscoop = true;
			}
			else if(stick1.GetRawButton(SCOOP_DOWN) || stick1.GetRawButton(SCOOP_UP)){
				autoscoop = false;
			}
			if(stick1.GetRawButton(GRIPPYS_DOWN)){
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
				else if(stick1.GetRawButton(SCOOP_BALLS)){
					runBrush = true;
				}
				if(stick1.GetRawButton(SCOOP_BALLS) || stick1.GetRawButton(SCOOP_BRIDGE)){
					if(scoopDown.Get()){
						scoopMotorValue = 0;
					}
					else if(stick1.GetRawButton(SCOOP_BRIDGE)){
						scoopMotorValue = 1;
					}
					else{
						scoopMotorValue = 0.4;
					}
				}
				else if(scoopUp.Get()){
					scoopMotorValue = 0;
				}
				else{
					scoopMotorValue = -0.85;
				}
			}
			else{
				//Manual scoop
				if(stick1.GetRawButton(SCOOP_DOWN)){
					if(scoopDown.Get()){
						scoopMotorValue = 0;
					}
					else{
						scoopMotorValue = 0.6;
					}
				}
				else if(stick1.GetRawButton(SCOOP_UP)){
					if(scoopUp.Get()){
						scoopMotorValue = 0;
					}
					else{
						scoopMotorValue = -0.6;
					}
				}
				else{
					scoopMotorValue = 0;	
				}
				if(stick2.GetRawButton(RUN_BRUSH_MOTOR)){
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
			if(stick2.GetRawButton(ELEVATOR_REVERSE) || stick2.GetRawButton(ELEVATOR_FOWARD)){
				if(stick2.GetRawButton(ELEVATOR_REVERSE)){
					elevator.Set(Relay::kReverse);
				}
				else if(stick2.GetRawButton(ELEVATOR_FOWARD)){
					elevator.Set(Relay::kForward);
				}
			}
			else if(ballsInLow > 0 || (ballsInHigh == 1 && ballsInLow > 0)){
				elevator.Set(Relay::kForward);
			}
			else{
				elevator.Set(Relay::kOff);
			}
			//Flywheel speed
			if(stick2.GetRawButton(KEY_BUTTON)){  
				desiredFlywheelSpeed = key;
			}
			else if(stick2.GetRawButton(REDUCE_FLYWHEEL_SPEED)){
				desiredFlywheelSpeed = (desiredFlywheelSpeed - 100);
			}
			else if(stick2.GetRawButton(INCREASE_FLYWHEEL_SPEED)){
				desiredFlywheelSpeed = (desiredFlywheelSpeed + 100);
			}
			flywheelspeed.SetSetpoint(desiredFlywheelSpeed);
			if(stick2.GetRawButton(FLYWHEELS_ON)){
				flywheelsOn = true;
			}
			else if(stick2.GetRawButton(FLYWHEELS_OFF)){
				flywheelsOn = false;
			}
			if(flywheelsOn){
				bottomWheelsMotor.Set((flywheelspeed.Get())/3000);
			}
			else{
				bottomWheelsMotor.Set(0);
			}
			//Actuators
			if(stick2.GetRawButton(FIRE_BUTTON) && !fireButton){
				stopwatch.Reset();
				stopwatch.Start();
				waitForLeaving = false;
				button1 = true;
			}
			if(stopwatch.Get() >= 0.25 || waitForLeaving == true){
				if(stopwatch.Get() >= 0.5 || waitForLeaving == true){
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
