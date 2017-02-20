#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include "WPILib.h"
#include "CANTalon.h"
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <thread>
#include <doubleSolenoid.h>
#include <AnalogPotentiometer.h>
#include <functional>
#include <AnalogInput.h>
#include <PIDSource.h>

class Robot: public frc::IterativeRobot {
private:

	const double EPSILON = 1E-1;
	double soloWait;	// Time it takes for the Solenoid to shift gears
	DoubleSolenoid *gearbox1;

	std::function<void()> autonomousProgram;

	AnalogInput *potPort;
	PIDController *gearController;
	PIDOutput *gearMotor;

	Joystick *leftJoyStick;
	Joystick *rightJoyStick;

	CANTalon *MotorR1;
	CANTalon *MotorR2;
	CANTalon *MotorR3;
	CANTalon *MotorL1;
	CANTalon *MotorL2;
	CANTalon *MotorL3;

	CANTalon *TopBallIntake;
	CANTalon *BottomBallIntake;

	Timer *AutoTimer;

	//double gearMechOffset = 2930.0;	//
	double positionUp = 3.44;		// Highest position
	double positionLoad = 3.48;		// Load position
	double positionDown = 3.56;		// Lowest position

	double autonMode;

	bool shouldStopGear = true;

	double thresh;		// Threshold, if joystick value is below this number, the motors will be set to 0;
	double throttlePre;	// Contains the raw value of the left joystick Y-axis
	double steerPre;	// Contains the raw value of the right joystick X-axis

	double throttle;	// Value used to set motors, derived from throttlePre
	double steer;		// Value used to adjust motors in order to turn, derived from steerPre
	int invert;			// value used to invert the motors by multiplying by throttle, (either 1 or -1)
	bool invertTest;	// Used to make sure the invert If statement doesn't run multiple times with one press

	bool soloTest;		// Used to make sure the Solenoid If statement doesn't run multiple times with one press

	bool isHighGear;	// States whether the robot is in high gear or low gear (true==high gear, false=low gear)
	bool isInverted;	// True if robot drive is Inverted, false if not inverted

	bool testButton3;
	bool testButton4;
	bool testButton5;
	double gearSetPoint;
	double gearPosition;
	double StartTime;	// ?????
	const double GEAR_UP_VELOCITY = -0.28;
	const double GEAR_DOWN_VELOCITY = 0.1;
	const double GEAR_HOLD = -0.15;

	double ballIntakeSpeed = 0.25;

	double roboRioAngle = 0.0;

	Accelerometer *accel;
	double lastTime = 0.0;
public:
	void RobotInit();
	void shiftHigh();
	void shiftLow();

	void AutonomousInit() override{ // Set the pointer for the autonomous program based on the dashboard value
		autonMode = SmartDashboard::GetNumber("Current Mode", 1);
		/*switch(autonMode){
		case 1:
			autonomousProgram = std::bind(Robot::auton1, this);
			break;
		case 2:
			autonomousProgram = std::bind(Robot::auton2, this);
			break;
		case 3:
			autonomousProgram = std::bind(Robot::auton3, this);
			break;
		default:
			autonomousProgram = std::bind(Robot::auton0, this);
		}*/

		AutoTimer->Reset();
		AutoTimer->Start();
	}
	void auton0();
	void auton1();
	void auton2();
	void auton3();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
};

void Robot::RobotInit() {
    //std::thread visionThread(VisionThread);
    //visionThread.detach();

	potPort = new AnalogInput(0);
	gearMotor = new CANTalon(7);
	gearController = new PIDController(15.0, 0.7, 8.5, potPort, gearMotor);
	gearController->Enable();
	SmartDashboard::PutNumber("P", gearController->GetP());
	SmartDashboard::PutNumber("I", gearController->GetI());
	SmartDashboard::PutNumber("D", gearController->GetD());

	SmartDashboard::PutNumber("Current Mode", autonMode);

	accel = new BuiltInAccelerometer(Accelerometer::kRange_4G);

	double accelY = accel->GetY();
	double accelZ = accel->GetZ();
	roboRioAngle = atan(accelY/accelZ);

    this -> autonMode = 0;

    this ->testButton3=true;
    this ->testButton4=true;
    this ->testButton5=true;

	this -> thresh = .05;		// Threshold, if joystick value is below this number, the motors will be set to 0;
	this -> throttle = 0;		// Value used to set motors, derived from throttlePre
	this -> steer = 0;			// Value used to adjust motors in order to turn, derived from steerPre
	this -> throttlePre = 0;	// Contains the raw value of the left joystick Y-axis
	this -> steerPre = 0;		// Contains the raw value of the right joystick X-axis
	this -> invert = -1;		//value used to invert the motors by multiplying by throttle, (either 1 or -1)
	this -> invertTest = true;	// Used to make sure the invert If statement doesn't run multiple times with one press

	this -> soloTest = true;	// Used to make sure the Solenoid If statement doesn't run multiple times with one press
	this -> isHighGear = true;	// States whether the robot is in high gear or low gear (true==high gear, false=low gear)
	soloWait = .25;				// Time it takes for the Solenoid to shift gears
	this -> StartTime = 0;		//??????

	this -> gearSetPoint = positionUp;

	leftJoyStick = new Joystick(0);
	rightJoyStick = new Joystick(1);

	MotorR1 = new CANTalon(1);
	MotorR2 = new CANTalon(2);
	MotorR3 = new CANTalon(3);
	MotorL1 = new CANTalon(4);
	MotorL2 = new CANTalon(5);
	MotorL3 = new CANTalon(6);

	TopBallIntake = new CANTalon(10);
	BottomBallIntake = new CANTalon(8);

	AutoTimer = new Timer();

	//gearMech = new AnalogPotentiometer(0,360,0);
	//gearMechOffset = gearMech->Get();
	//positionDown = gearMechOffset - 10;

	gearbox1=new DoubleSolenoid(9,4,5);

	MotorR1->SetControlMode(CANSpeedController::kFollower);
	MotorR1->Set(2);
	MotorR3->SetControlMode(CANSpeedController::kFollower);
	MotorR3->Set(2);

	MotorL1->SetControlMode(CANSpeedController::kFollower);
	MotorL1->Set(5);
	MotorL3->SetControlMode(CANSpeedController::kFollower);
	MotorL3->Set(5);

	MotorR2->Set(0);
	MotorL2->Set(0);
}

void Robot::shiftHigh(){//Puts the robot into High-Gear
	gearbox1->Set(DoubleSolenoid::Value::kForward);
	Wait(soloWait);
	gearbox1->Set(DoubleSolenoid::Value::kOff);
}

void Robot::shiftLow(){ //Puts the robot into Low-Gear
	gearbox1->Set(DoubleSolenoid::Value::kReverse);
	Wait(soloWait);
	gearbox1->Set(DoubleSolenoid::Value::kOff);
}


void Robot::auton0(){ // Default Autonomous mode does nothing
	this->MotorR2->Set(0.0);
}

void Robot::auton1(){
	// do stuff
}

void Robot::auton2(){
	// do stuff
}

void Robot::auton3(){
	// do stuff
}

void Robot::AutonomousPeriodic() {
	SmartDashboard::PutNumber("Accel X", accel->GetX());
	SmartDashboard::PutNumber("Accel Y", accel->GetY());
	SmartDashboard::PutNumber("Accel Z", accel->GetZ());
	this -> autonomousProgram();
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	SmartDashboard::PutBoolean("invert",isInverted);		//Displays the bool isInverted on the SmartDashboard
	SmartDashboard::PutNumber("throttle",throttlePre);	// Displays the raw value for the joystick on the Smart Dashboard
	SmartDashboard::PutNumber("gearSetPoint",gearSetPoint);
	//SmartDashboard::PutNumber("potentiometerValue",gearMech->Get());
	SmartDashboard::PutBoolean("HighGear",isHighGear);
	//gearSetPoint = SmartDashboard::GetNumber("gearSetPoint",20);

	throttlePre = leftJoyStick->GetY();
	steerPre = rightJoyStick->GetX();

	if(rightJoyStick->GetRawButton(1)){
		// Ball intake
		TopBallIntake->Set(ballIntakeSpeed);
		BottomBallIntake->Set(-(ballIntakeSpeed*2));
	}else if(leftJoyStick->GetRawButton(1)){
		// Ball outtake
		TopBallIntake->Set(ballIntakeSpeed);
		BottomBallIntake->Set(ballIntakeSpeed);
	}else{
		TopBallIntake->Set(0.0);
		BottomBallIntake->Set(0.0);
	}
	//When button is pressed the motors are inverted via int invert and bool isInverted represents if the motors are inverted
	if(rightJoyStick -> GetRawButton(2)&invertTest) {
		invert *= -1;
		invertTest = false;
		isInverted = !isInverted;
	}
	if(rightJoyStick -> GetRawButton(2) == false){
		invertTest = true;
	}

	//Shifts between low gear and high gear. bool isHighGear represents if it is in high gear (true) or low gear (False)
	if(leftJoyStick -> GetRawButton(2)&soloTest) {
		if(isHighGear)
		{
			shiftHigh();
			isHighGear=false;
		}
		else if(!isHighGear)
		{
			shiftLow();
			isHighGear=true;
		}
		soloTest=false;
	}
	if(!leftJoyStick -> GetRawButton(2)){
		soloTest=true;
	}

	//put to unload
	if(rightJoyStick -> GetRawButton(4)) {
		// Set to bottom
		gearSetPoint = positionDown;	}
	//put to upright
	else if(rightJoyStick -> GetRawButton(3)) {
		// Set to load
		gearSetPoint = positionLoad;
	}
	//put to loading
	else if(rightJoyStick -> GetRawButton(6)){
		// Set to top
		gearSetPoint = positionUp;
	}

	gearController->SetPID(SmartDashboard::GetNumber("P", 0),SmartDashboard::GetNumber("I", 0),SmartDashboard::GetNumber("D", 0));
	gearController->SetOutputRange(-0.2, 0.1);
	gearController->SetAbsoluteTolerance(.01);
	gearController->SetSetpoint(gearSetPoint);

	SmartDashboard::PutNumber("GearMotorPower", gearController->Get());
	SmartDashboard::PutNumber("GearSetpoint", gearController->GetSetpoint());
	SmartDashboard::PutNumber("gearController", gearController->GetError());
	SmartDashboard::PutNumber("GearPotentiometer", potPort->PIDGet());

	MotorR2->Set(throttlePre*invert - steerPre);
	MotorL2->Set(-throttlePre*invert - steerPre);

	frc::Wait(0.005);
}


START_ROBOT_CLASS(Robot)
