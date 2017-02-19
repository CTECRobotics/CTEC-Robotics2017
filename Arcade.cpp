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

const double EPSILON = 1E-1;
bool isInverted;	//True if robot drive is Inverted, false if not inverted
double soloWait;	// Time it takes for the Solenoid to shift gears
DoubleSolenoid *gearbox1;
class Robot: public frc::IterativeRobot {
public:


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

//	DigitalInput limitSwitch;

	CANTalon *gearMotor;


	Potentiometer *gearMech;
	Timer *AutoTimer;

	double gearMechOffset = 0.0;
	double &gearMax = gearMechOffset;
	double gearMin = 0.0f;

	double autonMode;

	bool auton1;		// booleans to control which autonomous codes are able to run
	bool auton2;		// basically a switch...
	bool auton3;

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

	static void VisionThread()
	{
		        cs::UsbCamera cameraF = CameraServer::GetInstance()->StartAutomaticCapture(0);
		        cameraF.SetResolution(320, 240);
		        cameraF.SetFPS(10);

		        cs::UsbCamera cameraR = CameraServer::GetInstance()->StartAutomaticCapture(1);
		        cameraR.SetResolution(320, 240);
		        cameraR.SetFPS(10);

		        cs::CvSink cvSink1 = CameraServer::GetInstance()->GetVideo(cameraF);
		        cs::CvSink cvSink2 = CameraServer::GetInstance()->GetVideo(cameraR);

		        cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);

		        cv::Mat source;
		        cv::Mat output;

		        while(true) {
		        	if(isInverted == false) {
		        cvSink1.GrabFrame(source);
		        cvtColor(source, output, cv::COLOR_BGR2GRAY);
		        outputStreamStd.PutFrame(output);
		        	} else {
				cvSink2.GrabFrame(source);
				cvtColor(source, output, cv::COLOR_BGR2GRAY);
				outputStreamStd.PutFrame(output);
		        	}
		        }
		    }

	void RobotInit() {
        //std::thread visionThread(VisionThread);
        //visionThread.detach();

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
		/**
		 * @brief AnalogPotentiometer objecy for the potentiometer
		 * @param first0 ID Number
		 * @param 110 Max value to return from potentiometer->Get()
		 * @param second0 offset
		 */

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
	//Puts the robot into High-Gear
	 void shiftHigh()
	{
		gearbox1->Set(DoubleSolenoid::Value::kForward);
		Wait(soloWait);
		gearbox1->Set(DoubleSolenoid::Value::kOff);
	}
	//Puts the robot into Low-Gear
	 void shiftLow()
	{
		gearbox1->Set(DoubleSolenoid::Value::kReverse);
		Wait(soloWait);
		gearbox1->Set(DoubleSolenoid::Value::kOff);
	}
	void AutonomousInit() override {
		autonMode = SmartDashboard::GetNumber("Current Mode", 1);
		if(autonMode == 1) {
			auton1 = true;
			auton2=false;
			auton3=false;
		} else if(autonMode == 2) {
			auton2 = true;
			auton1=false;
			auton3=false;
		} else if(autonMode == 3){
			auton3 = true;
			auton1=false;
			auton2=false;
		} else {

		}

		AutoTimer->Reset();
		AutoTimer->Start();
	}

	void AutonomousPeriodic() {
		/*double currentTime = AutoTimer->Get();
		double deltaTime = currentTime - lastTime;
		lastTime = currentTime;
		if (currentTime < 2){
			MotorL2->Set(-0.1);
			MotorR2->Set(0.1);
		}

		if(auton1 && AutoTimer->Get() < 5) {

				MotorL2->Set(-0.2);
			    MotorR2->Set(0.2);


		} else if (auton2 && AutoTimer->Get() < 5) {
				MotorL2->Set(0.2);
			    MotorR2->Set(-0.2);


		} else if(auton3 && AutoTimer->Get() < 5) {

				MotorL2->Set(-0.2);
			    MotorR2->Set(-0.2);
		}
		else {
			 MotorL2->Set(0);
			 MotorR2->Set(0);
		}

*/
	}
	void TeleopInit() {

	}

	void TeleopPeriodic() {
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
		/* Newish gear motor
		double gearMechVel = gearMech->Get();
		if (gearMechVel - gearSetPoint > EPSILON){
			//Move up
			gearMotor->Set(GEAR_UP_VELOCITY);
		}else if (gearMechVel - gearSetPoint < -EPSILON){
			// Move down
			gearMotor->Set(GEAR_DOWN_VELOCITY);

		}else{
			//Hold position
			gearMotor->Set(GEAR_HOLD);

		}*/
		//SmartDashboard::PutNumber("Potentiometer Thing", gearMechVel);

		/* Old Potentiometer stuff
		bool needToMoveGear = (abs(gearMechVel - gearSetPoint) > EPSILON);
		needToMoveGear =true;

		if (shouldStopGear){
			gearMotor->Set(this->GEAR_HOLD);
		}else if (needToMoveGear && gearSetPoint == gearMin){
			gearMotor->Set(this->GEAR_UP_VELOCITY);
		}else if (needToMoveGear && gearSetPoint == gearMax){
			gearMotor->Set(this->GEAR_DOWN_VELOCITY);
		}else if (needToMoveGear && gearSetPoint == gearMin+5){
			if(gearMechVel > gearSetPoint){
				gearMotor->Set(this->GEAR_UP_VELOCITY);
			}else{
				gearMotor->Set(this->GEAR_DOWN_VELOCITY);
			}
		}else if(!needToMoveGear){
			shouldStopGear = true;
		}*/

		SmartDashboard::PutNumber("Gear Motor Value", gearMotor->Get());


		//gearMotor->Set((gearMech->Get() - gearSetPoint)/110);

		MotorR2->Set(throttlePre*invert - steerPre);

		MotorL2->Set(-throttlePre*invert - steerPre);

		frc::Wait(0.005);
	}

	void TestPeriodic() {
	}


private:

};

START_ROBOT_CLASS(Robot)
