#include "WPILib.h"


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

	static const float fender ;
	static const float key ;
	int autostate;
	int ballsInHigh;
	int ballsInLow;
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

	void Drive();
	void Scoop();
	void Elevator();
	void Shoot();
	float ConvertAxis(float input);

public:
	RobotDemo(void);

	void Autonomous(void);

	void OperatorControl(void);
};
