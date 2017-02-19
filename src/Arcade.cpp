/*
 * Arcade.cpp
 *
 *  Created on: Feb 19, 2017
 *      Author: ImMilesAhead
 */

#include "Arcade.h"

void Robot::RobotInit() {
    //std::thread visionThread(VisionThread);
    //visionThread.detach();
	potPort = new AnalogInput(1);
	gearController = new PIDController(0.1, 0, 0.0, &potPort, &gearMotor);
	SmartDashboard::PutNumber("Current Mode", autonMode);

	accel = new BuiltInAccelerometer(Accelerometer::kRange_4G);

	double accelY = accel->GetY();
	double accelZ = accel->GetZ();
	roboRioAngle = atan(accelY/accelZ);


	AnalogInput::SetSampleRate(62500);
	this -> auton1 = false;
	this -> auton2 = false;
	this -> auton3 = false;
    this -> autonMode = 0;

    this ->testButton3=true;
    this ->testButton4=true;
    this ->testButton5=true;

	this -> thresh = .05;		// Threshold, if joystick value is below this number, the motors will be set to 0;
	this -> throttle = 0;		// Value used to set motors, derived from throttlePre
	this -> steer = 0;		// Value used to adjust motors in order to turn, derived from steerPre
	this -> throttlePre = 0;	// Contains the raw value of the left joystick Y-axis
	this -> steerPre = 0;		// Contains the raw value of the right joystick X-axis
	this -> invert = -1;		//value used to invert the motors by multiplying by throttle, (either 1 or -1)
	this -> invertTest = true;	// Used to make sure the invert If statement doesn't run multiple times with one press

	this -> soloTest = true;	// Used to make sure the Solenoid If statement doesn't run multiple times with one press
	this -> isHighGear = true;	// States whether the robot is in high gear or low gear (true==high gear, false=low gear)
	soloWait = .25;		// Time it takes for the Solenoid to shift gears
	this -> StartTime = 0;		//??????

		//limitSwitch=new DigitalInput(1);
	this ->gearSetPoint=0;
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


	gearMotor = new CANTalon(7);
	AutoTimer = new Timer();

	gearMech = new AnalogPotentiometer(0,360,0);
	gearMechOffset = gearMech->Get();
	gearMin = gearMechOffset - 10;

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

void Robot::AutonomousInit(){ // Set the pointer for the autonomous program based on the dashboard value
	autonMode = SmartDashboard::GetNumber("Current Mode", 1);
	switch(autonMode){
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
	}

	AutoTimer->Reset();
	AutoTimer->Start();
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
	SmartDashboard::PutNumber("potentiometerValue",gearMech->Get());
	SmartDashboard::PutBoolean("HighGear",isHighGear);
	gearSetPoint=SmartDashboard::GetNumber("gearSetPoint",20);
	throttlePre = leftJoyStick->GetY();
	steerPre = rightJoyStick->GetX();

	throttle = pow(2, throttlePre) - 1;	//Gets value for motors based on input from joystick ((2^ValueFromJoystick)-1)
	steer = pow(2, steerPre) - 1;


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
	if(rightJoyStick -> GetRawButton(3)) {
		// Set to top
		// which is motorgear min
		gearMotor->Set(0.0);
		gearSetPoint = gearMin;
	}
	//put to upright
	else if(rightJoyStick -> GetRawButton(4)) {
		// Set to bottom
		// which is motor gear max
		gearMotor->Set(-0.35);
		gearSetPoint = gearMax;
	}
	//put to loading
	else if(rightJoyStick -> GetRawButton(5)){
		// Set to middle
		// which is motorgearMax-5
		gearSetPoint = gearMax - 5;
	}else{
		gearMotor->Set(-0.2);
	}
	gearController->SetPID(SmartDashboard::GetNumber("P", 1),SmartDashboard::GetNumber("I", 1),SmartDashboard::GetNumber("D", 1));
	gearController->SetSetpoint(gearSetPoint);
	SmartDashboard::PutNumber("Gear Motor Value", gearMotor->Get());

	MotorR2->Set(throttlePre*invert - steerPre);

	MotorL2->Set(-throttlePre*invert - steerPre);

	frc::Wait(0.005);
}
