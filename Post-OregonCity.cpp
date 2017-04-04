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
#include <Timer.h>
#include <ctime>
#include <Ultrasonic.h>

#define X 0
#define Y 1
#define Z 2

class Robot: public frc::IterativeRobot {
private:

	// BEGIN Accelerometer
	Accelerometer *accel;
	bool autonomousFlag = true;
	double forwardVelocityAuto = 0.5;


	double deltaTime = 0.0;
	int FPS = 0;
	int seconds = 0;
	double mg[3] = {0.0, 0.f, 0.0};         // Using --std=c++11
	double &forward = mg[Z];  // Assuming the Z axis is forward relative to the roborio
	double &up = mg[Y];       // Assuming the Y axis is up relative to the roborio
	double theta = 0.0f;
	                        // X     Y     Z
	double velocity[3] = {0.0, 0.0, 0.0};   // Using --std=c++11
	double position[3] = {0.0, 0.0, 0.0};   // Using --std=c++11

	// END Accelerometer

	const double EPSILON = 1E-1;
	double soloWait;	// Time it takes for the Solenoid to shift gears
	DoubleSolenoid *gearbox1;

	//std::function<void()> autonomousProgram;

	double turnValue;

	Joystick *leftJoyStick;
	Joystick *rightJoyStick;

	CANTalon *MotorR1;
	CANTalon *MotorR2;
	CANTalon *MotorR3;
	CANTalon *MotorL1;
	CANTalon *MotorL2;
	CANTalon *MotorL3;

    CANTalon *Wench1;
    CANTalon *Wench2;
    const double WENCH_POWER = 0.5;


	CANTalon *TopBallIntake;
	CANTalon *BottomBallIntake;
	Ultrasonic *AutoRangeFinder;
	Timer *AutoTimer;

	// BEGIN Potentiometer and Gear Motor
	AnalogInput *potPort;
	PIDController *gearController;
	PIDOutput *gearMotor;
	double positionUp = 0.0;		//  0.12 difference from down  3.44;		// Highest position
	double positionLoad = 0.0;		// 0.0225 diff from up 3.4625;			// Load position
	double positionDown = 0.0;	//3.56;						// Lowest position
	const double MAX_POWER = 0.1;
	const double MIN_POWER = -0.3;
	// END Potentiometer and Gear Motor


	double Autotimer1;
	double InitAutoTimer;
	//double autonMode;

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

public:

	static void VisionThread(){
		cs::UsbCamera camera = CameraServer::GetInstance() -> StartAutomaticCapture(0);
		camera.SetResolution(320, 240);
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(camera);
		cs::CvSource s = CameraServer::GetInstance()->PutVideo("Gray", 320, 240);

		//cs::CvSource outputStreamStd =  CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
		cv::Mat source;
		cv::Mat output;
		while(true) {
			cvSink.GrabFrame(source);
			cvtColor(source, output, cv::COLOR_BGR2GRAY);
			s.PutFrame(output);
		}
	}
	void RobotInit();
	void shiftHigh();
	void shiftLow();
	void placeGear();
	void adjustGear(int adjust);
//	void auton0();
//	void auton1();
//	void auton2();
//	void auton3();
	void Autonomous();
	void AutonomousPeriodic();
	void AutonomousInit();
	void TeleopInit();
	void TeleopPeriodic();
	void gear_aton();
	void baseline_aton();
};

void Robot::RobotInit() {
    std::thread visionThread(VisionThread);
    visionThread.detach();
	potPort = new AnalogInput(0);
	gearMotor = new CANTalon(7);
	gearController = new PIDController(15.0, 0.7, 8.5, potPort, gearMotor);
	gearController->Enable();
	SmartDashboard::PutNumber("P", gearController->GetP());
	SmartDashboard::PutNumber("I", gearController->GetI());
	SmartDashboard::PutNumber("D", gearController->GetD());
	SmartDashboard::PutNumber("turn_Deg",0);
	positionUp = potPort->PIDGet();
	positionDown = positionUp + 0.12;
	positionLoad = positionUp + 0.0560;
	gearSetPoint = positionUp;
	gearController->SetSetpoint(positionUp);
	//SmartDashboard::PutNumber("Current Mode", autonMode);


	// BEGIN Accelerometer initialization
	accel = new BuiltInAccelerometer(Accelerometer::kRange_4G);

	mg[X] = accel->GetX();
	mg[Y] = accel->GetY();
	mg[Z] = accel->GetZ();
	/*
	 * forward is a refrence to which ever axis points forward relative to the roborio
	 * up is a refrence to which ever axis points up relative to the roborio
	 */
	theta = atan(forward/up);
	SmartDashboard::PutNumber("Theta", theta);
	SmartDashboard::PutNumber("Cos(Theta)", cos(theta));
	SmartDashboard::PutNumber("Sin(Theta)", sin(theta));
	// END Acccelerometer Initialization

    //this -> autonMode = 1;

    this ->testButton3=true;
    this ->testButton4=true;
    this ->testButton5=true;

    this ->Autotimer1 = 0;
    this ->InitAutoTimer = 0;

	this -> thresh = .05;		// Threshold, if joystick value is below this number, the motors will be set to 0;
	this -> throttle = 0;		// Value used to set motors, derived from throttlePre
	this -> steer = 0;			// Value used to adjust motors in order to turn, derived from steerPre
	this -> throttlePre = 0;	// Contains the raw value of the left joystick Y-axis
	this -> steerPre = 0;		// Contains the raw value of the right joystick X-axis
	this -> invert = 1;		//value used to invert the motors by multiplying by throttle, (either 1 or -1)
	this -> invertTest = true;	// Used to make sure the invert If statement doesn't run multiple times with one press

	this -> soloTest = true;	// Used to make sure the Solenoid If statement doesn't run multiple times with one press
	this -> isHighGear = false;	// States whether the robot is in high gear or low gear (true==high gear, false=low gear)
	soloWait = .25;				// Time it takes for the Solenoid to shift gears
	this -> StartTime = 0;		//??????

	this -> gearSetPoint = positionUp;

	this->turnValue=0;
	leftJoyStick = new Joystick(0);
	rightJoyStick = new Joystick(1);

	MotorR1 = new CANTalon(1);
	MotorR2 = new CANTalon(2);
	MotorR3 = new CANTalon(3);
	MotorL1 = new CANTalon(4);
	MotorL2 = new CANTalon(5);
	MotorL3 = new CANTalon(6);

    Wench1 = new CANTalon(11);
    Wench2 = new CANTalon(12);

	TopBallIntake = new CANTalon(10);
	BottomBallIntake = new CANTalon(8);

	AutoRangeFinder = new Ultrasonic(1,1);
	AutoRangeFinder->SetAutomaticMode(true);

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

void Robot::shiftLow(){//Puts the robot into High-Gear
	gearbox1->Set(DoubleSolenoid::Value::kForward);
	Wait(soloWait);
	gearbox1->Set(DoubleSolenoid::Value::kOff);
	isHighGear=!isHighGear;
}
void Robot::adjustGear(int adjust){
		positionUp = positionUp+adjust*-.035/16;
		positionDown = positionUp + 0.12;
		positionLoad = positionUp + 0.0560;
		gearController->SetSetpoint(positionUp);
}
void Robot::shiftHigh(){ //Puts the robot into Low-Gear
	gearbox1->Set(DoubleSolenoid::Value::kReverse);
	Wait(soloWait);
	gearbox1->Set(DoubleSolenoid::Value::kOff);
	isHighGear=!isHighGear;
}
void Robot::placeGear()
{
	while(true)
	{
	turnValue=SmartDashboard::GetNumber("turn_Deg", 0);
	turnValue/=20;
	if(abs(turnValue>.4))
	{
		MotorR2->Set(turnValue);
		MotorL2->Set(turnValue);
	}
	else if(turnValue>0)
	{
		MotorR2->Set(.6);
		MotorL2->Set((.6-turnValue)*-1);
	}
	else if (turnValue<0)
	{
		MotorR2->Set(.6-turnValue);
		MotorL2->Set(-.6);
	}
	else
	{
		while(AutoRangeFinder->GetRangeInches()>10)
		{
			MotorR2->Set(.35);
			MotorL2->Set(-.35);
		}
	}
	MotorR2->Set(0);
	MotorL2->Set(0);
}
}
/*
void Robot::auton0(){ // Default Autonomous mode does nothing
	this->MotorR2->Set(0.0);
	this->MotorL2->Set(0.0);
}

void Robot::auton1(){
	// do stuff

	}
}

void Robot::auton2(){
	// do stuff
}

void Robot::auton3(){
	// do stuff
}
*/

	void Robot::AutonomousInit() {
		int Mode = (int)SmartDashboard::GetNumber("Current Mode", 1);
		switch(Mode){
		case 1 :
			shiftLow();
			break;
		case 2 :
			shiftHigh();
			break;
		default :
			shiftLow();
			break;
		}
		AutoTimer->Reset();
		AutoTimer->Start();
		autonomousFlag = true;
	}

void Robot::AutonomousPeriodic() {
	int Mode = (int)SmartDashboard::GetNumber("Current Mode", 1);
	switch(Mode){
	case 1 :
		Robot::gear_aton();
		break;
	case 2 :
		Robot::baseline_aton();
		break;
	default:
		Robot::baseline_aton();
		break;
	}

}
void Robot::TeleopInit() {
	if(isHighGear){
			// do nothing
		} else {
			shiftHigh();
		}
}
void Robot::baseline_aton(){
	double currentTime = AutoTimer->Get();
		if(currentTime < 2.5){
	        // Drive Forward
	        MotorR2->Set(-forwardVelocityAuto);
	        MotorL2->Set(forwardVelocityAuto);
	    }   else{
	        if(this->autonomousFlag){
	            this->autonomousFlag = false;
	            MotorR2->Set(0.0);
	            MotorL2->Set(0.0);
	            gearSetPoint = positionUp;
	            // drop gearController
	        }
	    }
		 gearController->SetPID(SmartDashboard::GetNumber("P", 0),SmartDashboard::GetNumber("I", 0),SmartDashboard::GetNumber("D", 0));
				  		gearController->SetOutputRange(MIN_POWER, MAX_POWER);
				  		gearController->SetAbsoluteTolerance(.01);
				  		gearController->SetSetpoint(gearSetPoint);

				  		SmartDashboard::PutNumber("GearMotorPower", gearController->Get());
				  		SmartDashboard::PutNumber("GearSetpoint", gearController->GetSetpoint());
				  		SmartDashboard::PutNumber("gearController", gearController->GetError());
				  		SmartDashboard::PutNumber("GearPotentiometer", potPort->PIDGet());


	}
void Robot::gear_aton() {
	double currentTime = AutoTimer->Get();
		  if(currentTime<3){
			  gearSetPoint = positionUp;
			  MotorR2->Set(-.37);
			  MotorL2->Set(.37);
		  }
		  else if(currentTime<4){
			  gearSetPoint = positionDown;
			  MotorR2->Set(0);
			  MotorL2->Set(0);
		  }
		  else if(currentTime<5)
		  {
			  MotorR2->Set(.2);
			  MotorL2->Set(-.2);
		  }
		  else if(currentTime<6)
		  {
			  MotorR2->Set(-.2);
			  MotorL2->Set(.2);
		  }
		  else if(currentTime<7.5){
			  MotorR2->Set(.2);
			  MotorL2->Set(-.2);
		  }
		  else{
			  MotorR2->Set(0);
			  MotorL2->Set(0);
		  }
		  gearController->SetPID(SmartDashboard::GetNumber("P", 0),SmartDashboard::GetNumber("I", 0),SmartDashboard::GetNumber("D", 0));
		  		gearController->SetOutputRange(MIN_POWER, MAX_POWER);
		  		gearController->SetAbsoluteTolerance(.01);
		  		gearController->SetSetpoint(gearSetPoint);

		  		SmartDashboard::PutNumber("GearMotorPower", gearController->Get());
		  		SmartDashboard::PutNumber("GearSetpoint", gearController->GetSetpoint());
		  		SmartDashboard::PutNumber("gearController", gearController->GetError());
		  		SmartDashboard::PutNumber("GearPotentiometer", potPort->PIDGet());
}
void Robot::TeleopPeriodic() {
	//this->Autonomous();
	/*SmartDashboard::PutBoolean("invert",isInverted);		//Displays the bool isInverted on the SmartDashboard
	SmartDashboard::PutNumber("throttle",throttlePre);	// Displays the raw value for the joystick on the Smart Dashboard
	SmartDashboard::PutNumber("gearSetPoint",gearSetPoint);*/
	//SmartDashboard::PutNumber("potentiometerValue",gearMech->Get());
	SmartDashboard::PutBoolean("HighGear",isHighGear);
	//gearSetPoint = SmartDashboard::GetNumber("gearSetPoint",20);
	SmartDashboard::PutNumber("Distance", AutoRangeFinder->GetRangeInches());
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
    if(leftJoyStick -> GetRawButton(6)){
        Wench1->Set(WENCH_POWER);
        Wench2->Set(WENCH_POWER);
    }else if(leftJoyStick->GetRawButton(4)){
        Wench1->Set(-WENCH_POWER);
        Wench2->Set(-WENCH_POWER);
    }
    //MAXIMUM OVERCLIMB!!!!
    else if(leftJoyStick->GetRawButton(5)){
            Wench1->Set(2*WENCH_POWER);
            Wench2->Set(2*WENCH_POWER);
        }
    else{
        Wench1->Set(0.0);
        Wench2->Set(0.0);
    }
    if(rightJoyStick -> GetRawButton(9))
    {
    	adjustGear(1);
    }
    if(rightJoyStick -> GetRawButton(11))
    {
    	adjustGear(-1);
    }
	//Shifts between low gear and high gear. bool isHighGear represents if it is in high gear (true) or low gear (False)
	if(leftJoyStick -> GetRawButton(2)&soloTest) {
		if(isHighGear)
		{
			shiftHigh();
		}
		else if(!isHighGear)
		{
			shiftLow();
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
	gearController->SetOutputRange(MIN_POWER, MAX_POWER);
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
