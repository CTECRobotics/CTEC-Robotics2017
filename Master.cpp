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
#include <doubleSolenoid.h


bool isInverted;


class Robot: public frc::IterativeRobot {
public:

	Joystick *leftJoyStick;
	Joystick *rightJoyStick;

	CANTalon *MotorR1;
	CANTalon *MotorR2;
	CANTalon *MotorL1;
	CANTalon *MotorL2;
	
	DoubleSolenoid *gearbox1;

	Timer *AutoTimer;

	double thresh;
	double throttlePre;
	double steerPre;

	double throttle;
	double steer;
	int invert;
	bool invertTest;
	
	bool soloTest;
	double soloWait;
	bool isHighGear;
	
	double StartTime;
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
        std::thread visionThread(VisionThread);
        visionThread.detach();

		this -> thresh = .05;
		this -> throttle = 0;
		this -> steer = 0;
		this -> throttlePre = 0;
		this -> steerPre = 0;
		this -> invert = 1;
		this -> invertTest = true;
		
		this -> soloTest = true;
		this -> isHighGear=true;
		this -> soloWait = .25;
		this -> StartTime = 0;
		invertTest = true;

		leftJoyStick = new Joystick(0);
		rightJoyStick = new Joystick(1);

		MotorR1 = new CANTalon(2);
		MotorR2 = new CANTalon(1);
		MotorL1 = new CANTalon(3);
		MotorL2 = new CANTalon(4);
		AutoTimer = new Timer();
		
		gearbox1=new DoubleSolenoid(5,0,1);

		MotorR1->SetControlMode(CANSpeedController::kFollower);
		MotorR1->Set(1);

		MotorL1->SetControlMode(CANSpeedController::kFollower);
		MotorL1->Set(4);

		MotorR2->Set(0);
		MotorL2->Set(0);
	}

	void AutonomousInit() override {
		 AutoTimer->Reset();
		 AutoTimer->Start();
	}

	void AutonomousPeriodic() {

		if(AutoTimer->Get() < 5) {
			MotorL2->Set(-0.2);
		    MotorR2->Set(0.2);
		} else {
			MotorL2->Set(0);
			MotorR2->Set(0);
		}

	}
	void TeleopInit() {

	}

	void TeleopPeriodic() {

		throttlePre = leftJoyStick->GetY();
		steerPre = rightJoyStick->GetX();

		throttle = pow(2, throttlePre) - 1;
		steer = pow(2, steerPre) - 1;

		SmartDashboard::PutNumber("throttle",throttlePre);

		if(rightJoyStick -> GetRawButton(2)&invertTest) {
			invert *= -1;
			invertTest = false;
			isInverted = !isInverted;
		}
		if(rightJoyStick -> GetRawButton(2) == false){
			invertTest = true;
		}
		
		if(leftJoyStick -> GetRawButton(2)&soloTest) {
				if(isHighGear)
				{
					
					gearbox1->Set(DoubleSolenoid::Value::kForward);
					Wait(waitvar);
					gearbox1->Set(DoubleSolenoid::Value::kOff);
					isHighGear=false;
					
				}
				else if(!isHighGear)
				{
					
					gearbox1->Set(DoubleSolenoid::Value::kReverse);
					Wait(waitvar);
					gearbox1->Set(DoubleSolenoid::Value::kOff);
					isHighGear=true;
					
				}
				soloTest=false;
			}

			if(!leftJoyStick -> GetRawButton(2)){
				soloTest=true;
			}
		
		
		SmartDashboard::PutBoolean("invert",isInverted);

		MotorR2->Set(throttlePre*invert + steerPre);
		MotorL2->Set(-throttlePre*invert + steerPre);

		frc::Wait(0.005);
	}
	
	void TestPeriodic() {
	}

private:

};

START_ROBOT_CLASS(Robot)
