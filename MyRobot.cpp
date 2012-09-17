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
	int ballsinhigh;
	int ballsinlow;
	float xoutput;
	float youtput;
	float twistoutput;
	float desiredflywheelspeed;
	float PIDOutput;
	float scoopmotorvalue;
	bool launcheroutset;
	bool launcherinset;
	bool waitforleaving;
	bool firebutton;
	bool flywheelson;
	bool wheelsready;
	bool autoscoop;
	bool runbrush;
	bool ballintop;

	Joystick stick1;
	Joystick stick2;
	Jaguar ScoopMotor;
	Jaguar RearLeftMotor;
	Jaguar RearRightMotor;//inverted
	Jaguar FrontLeftMotor;
	Jaguar FrontRightMotor;//inverted
	Relay BrushMotor;
	Relay Elevator;
	Relay compressor;
	Victor BottomWheelsMotor;
	Solenoid LauncherIn;
	Solenoid LauncherOut;
	Solenoid WheelsDown;
	DigitalInput LowLightSensor;
	DigitalInput HighLightSensor;
	DigitalInput ScoopUp;
	DigitalInput ScoopDown;
	DigitalInput CompressorSwitch;
	Encoder LeftWheels;
	Encoder RightWheels;
	Encoder BottomWheels;
	PIDController flywheelspeed;
	Timer stopwatch;
	Timer stopwatch1;
	AxisCamera Camera;	//Initialize me
	RobotDrive myRobot; // robot drive system

	

public:
	RobotDemo(void):	//these must be intialized in the same order
		autostate(0),
		ballsinhigh(0), 
		ballsinlow(0),
		xoutput(0),
		youtput(0),
		twistoutput(0),
		desiredflywheelspeed(fender),
		PIDOutput(0),
		scoopmotorvalue(0),
		launcheroutset(false),
		launcherinset(false),
		waitforleaving(true),
		firebutton(false),
		flywheelson(false),
		wheelsready(false),
		autoscoop(false),
		runbrush(false),
		ballintop(false),		
		stick1(1),		// as they are declared above.
		stick2(2),
		ScoopMotor(1,5),
		RearLeftMotor(2,2),
		RearRightMotor(1,2),//inverted
		FrontLeftMotor(2,1),
		FrontRightMotor(1,1),//inverted
		BrushMotor(2,2),
		Elevator(2,1),
		compressor(1,4),
		BottomWheelsMotor(1,8),
		LauncherIn(1,3),
		LauncherOut(1,2),
		WheelsDown(1,1),
		LowLightSensor(2,7),
		HighLightSensor(2,8),
		ScoopUp(1,3),
		ScoopDown(1,4),
		CompressorSwitch(2,9),
		LeftWheels(2,1,2,2),
		RightWheels(1,1,1,2),
		BottomWheels(2,5,2,6),
		stopwatch(),
		stopwatch1(),
		flywheelspeed(3,.005,.0001,&BottomWheels,PIDOutput),
		Camera(),
		myRobot(&FrontLeftMotor, &RearLeftMotor, &FrontRightMotor, &RearRightMotor)
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
		WheelsDown.Set(true);
		LeftWheels.Reset();
		RightWheels.Reset();
		while(!IsOperatorControl()){
			while(true){		//must run for whole autonomous
				BottomWheelsMotor.Set((flywheelspeed.Get())/3000);
				if(flywheelspeed.Get()<= 2550 && flywheelspeed.Get() >= 2200){
					wheelsready = true;
				} else {
					wheelsready = false;
				}
			}
			while(autostate == 0){//shoots first ball
				if(wheelsready == true){
					Wait(.5);
					LauncherIn.Set(true);
					LauncherOut.Set(false);
					Wait(.25);
					LauncherOut.Set(true);
					LauncherIn.Set(false);
					wheelsready = false;
					autostate = 1;
				}
			}
			while(autostate == 1){//moves second ball into position
				Elevator.Set(Relay::kOn);
				Wait(2);
				Elevator.Set(Relay::kOff);
				autostate = 2;
			}
			while(autostate == 2){//shoots second ball
				if(wheelsready == true){
					Wait(.5);
					LauncherIn.Set(true);
					LauncherOut.Set(false);
					Wait(.25);
					LauncherOut.Set(true);
					LauncherIn.Set(false);
					wheelsready = false;
					autostate = 3;
				}
			}
			while(autostate == 3 && stick2.GetThrottle() > 2){	//Drive to bridge
				if(LeftWheels.Get() >= 3690 && RightWheels.Get() >= 3690){
					autostate = 4;
				}
				myRobot.TankDrive(.5, (LeftWheels.Get() - RightWheels.Get())*.001 + .5);
			}
			stopwatch.Reset();
			stopwatch.Start();
			while (autostate == 4 && stick2.GetThrottle() > 2){ //Tip bridge
				if(stopwatch.Get() >= 3){
					autostate = 5;
				}
				if(!ScoopDown.Get()){
					ScoopMotor.Set(1);
				}
				else{
					ScoopMotor.Set(0);
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
					ScoopMotor.Set(0);
					autostate = 7;
				}
				if(!ScoopUp.Get()){
					ScoopMotor.Set(-.6);
				}
				else{
					ScoopMotor.Set(0);
					autostate = 7;
				}
			}
			stopwatch.Stop();
			stopwatch.Reset();
		}
	}

	void OperatorControl(void)
	{
		WheelsDown.Set(false);
		GetWatchdog().SetEnabled(false);
		while (IsOperatorControl())
		{
			//Sensitivity of Joystick
			//X
			if(stick1.GetX() >= .05){
				xoutput = pow((stick1.GetX()*.05), 2);
			}
			else if(stick1.GetX() <= -.05){
				xoutput = pow((stick1.GetX()*.05), 2)*-1;
			}
			else{
				xoutput = 0;
			}
			//Y
			if(stick1.GetY() >= .05){
				youtput = pow((stick1.GetY()*.05), 2);
			}
			else if(stick1.GetY() <= -.05){
				youtput = pow((stick1.GetY()*.05), 2)*-1;
			}
			else{
				youtput = 0;
			}
			//Twist
			if(stick1.GetTwist() >= .05){
				twistoutput = pow((stick1.GetTwist()*.05), 2);
			}
			else if(stick1.GetX() <= -.05){
				twistoutput = pow((stick1.GetTwist()*.05), 2)*-1;
			}
			else{
				twistoutput = 0;
			}
			if(stick1.GetThrottle() >= 0){
				twistoutput = twistoutput/2;
			}
			else{
				twistoutput = twistoutput/1.5;
			}
			//Grippy Deployment
			if(stick1.GetRawButton(2)){
				xoutput = 0;
				WheelsDown.Set(true);
			}
			else{
				WheelsDown.Set(false);
			}
			//Drive with Mecanum style
			myRobot.MecanumDrive_Cartesian(xoutput, youtput, twistoutput);
			//checks for current scoop mode
			if(stick1.GetRawButton(12) || stick1.GetRawButton(11)){
				autoscoop = true;
			}
			else if(stick1.GetRawButton(7) || stick1.GetRawButton(9)){
				autoscoop = false;
			}
			if(stick1.GetRawButton(2)){
				WheelsDown.Set(true);
			}
			else{
				WheelsDown.Set(false);
			}
			//Autoscoop
			if(autoscoop == true){
				if(ScoopUp.Get()){
					runbrush = false;
				}
				else if(stick1.GetRawButton(7)){
					runbrush = true;
				}
				if(stick1.GetRawButton(7) || stick1.GetRawButton(9)){
					if(ScoopDown.Get()){
						scoopmotorvalue = 0;
					}
					else if(stick1.GetRawButton(9)){
						scoopmotorvalue = 1;
					}
					else{
						scoopmotorvalue = .4;
					}
				}
				else if(ScoopUp.Get()){
					scoopmotorvalue = 0;
				}
				else{
					scoopmotorvalue = -.85;
				}
			}
			else{
				//Manual scoop
				if(stick1.GetRawButton(11)){
					if(ScoopDown.Get()){
					scoopmotorvalue = 0;
					}
					else{
						scoopmotorvalue = .6;
					}
				}
				else if(stick1.GetRawButton(12)){
					if(ScoopUp.Get()){
						scoopmotorvalue = 0;
					}
					else{
						scoopmotorvalue = -.6;
					}
				}
				else{
					scoopmotorvalue = 0;	
				}
				if(stick2.GetRawButton(8)){
					runbrush = true;
				}
				else{
					runbrush = false;
				}
			}
			//Ball Position
			if(LowLightSensor.Get()){
				ballsinlow++;
			}
			if(HighLightSensor.Get()){
				stopwatch1.Reset();
				stopwatch1.Start();
			}
			if(stopwatch1.Get() >= 2.5){
				ballsinhigh++;//FIX ME
				ballsinlow--;
				stopwatch1.Stop();
				stopwatch1.Reset();
			}
			if(ballsinhigh > 0){
				ballintop = true;
			}
			else{
				ballintop = false;
			}
			//Elevator
			if(stick2.GetRawButton(10) || stick2.GetRawButton(11)){
				if(stick2.GetRawButton(10)){
					Elevator.Set(Relay::kReverse);
				}
				else if(stick2.GetRawButton(11)){
					Elevator.Set(Relay::kForward);
				}
				else{
					Elevator.Set(Relay::kOff);
				}
			}
			else if(ballsinlow > 0 || (ballsinhigh == 1 && ballsinlow > 0)){
				Elevator.Set(Relay::kForward);
			}
			else{
				Elevator.Set(Relay::kOff);
			}
			//Flywheel speed
			if(stick2.GetRawButton(3)){  //ME TOO
				desiredflywheelspeed = key;
			}
			else if(stick2.GetRawButton(4)){
				desiredflywheelspeed = (desiredflywheelspeed - 100);
			}
			else if(stick2.GetRawButton(5)){
				desiredflywheelspeed = (desiredflywheelspeed + 100);
			}
			flywheelspeed.SetSetpoint(desiredflywheelspeed);
			if(stick2.GetRawButton(9)){
				flywheelson = true;
			}
			else if(stick2.GetRawButton(6)){
				flywheelson = false;
			}
			if(flywheelson){
				BottomWheelsMotor.Set((flywheelspeed.Get())/3000);
			}
			else{
				BottomWheelsMotor.Set(0);
			}
			//Actuators
			if(stick2.GetRawButton(1) && !firebutton){
				stopwatch.Reset();
				stopwatch.Start();
				waitforleaving = false;
			}
			if(stopwatch.Get() >= .25 || waitforleaving == true){
				if(stopwatch.Get() >= .5 || waitforleaving == true){
					launcheroutset = false;
					launcherinset = false;
				}
				else{
					launcheroutset = true;
					launcherinset = false;
				}
			}
			else{
				launcheroutset = false;
				launcherinset = true;
			}
			LauncherIn.Set(launcherinset);
			LauncherOut.Set(launcheroutset);
			if(stopwatch.Get() >= 5){
				stopwatch.Stop();
			}
			//Makes 1 time button work
			if(stick2.GetRawButton(1)){
				firebutton = true;
			}
			else{
				firebutton = false;
			}
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

