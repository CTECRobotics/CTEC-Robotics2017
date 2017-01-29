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

class Robot: public frc::IterativeRobot {
public:


	Joystick *leftJoyStick;
	Joystick *rightJoyStick;

	CANTalon *MotorR1;
	CANTalon *MotorR2;
	CANTalon *MotorL1;
	CANTalon *MotorL2;
	double thresh;
	double throttlePre;
	double steerPre;

	double throttle;
	double steer;
	int invert;
	bool invertTest;

	double StartTime;

	void RobotInit() {
		this -> thresh = .05;
		this -> throttle = 0;
		this -> steer = 0;
		this -> throttlePre = 0;
		this -> steerPre = 0;
		this -> invert = 1;
		this -> invertTest = true;
		this -> StartTime = 0;

		leftJoyStick = new Joystick(0);
		rightJoyStick = new Joystick(1);

		MotorR1 = new CANTalon(1);
		MotorR2 = new CANTalon(2);
		MotorL1 = new CANTalon(3);
		MotorL2 = new CANTalon(4);

		MotorR1->Set(0);
		MotorR2->Set(0);
		MotorL1->Set(0);
		MotorL2->Set(0);
	}

	void AutonomousInit() override {
		 StartTime = GetFPGATime();
	}

	void AutonomousPeriodic() {
		double CurrentTime = GetFPGATime();

		if(CurrentTime - StartTime < 5000) {
			MotorL1->Set(-0.2);
			MotorL2->Set(-0.2);
			MotorR1->Set(0.2);
		    MotorR2->Set(0.2);
		    Wait(0.005);
		} else {
			MotorL1->Set(0);
			MotorL2->Set(0);
			MotorR1->Set(0);
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

		SmartDashboard::PutNumber("DB/Slider 0", leftJoyStick -> GetY());
		SmartDashboard::PutNumber("DB/Slider 1", rightJoyStick -> GetY());

		SmartDashboard::PutNumber("DB/Slider 2", throttlePre);
		SmartDashboard::PutNumber("DB/Slider 3", steerPre);

		if(rightJoyStick -> GetRawButton(2)&invertTest) {
			invert *= -1;
			invertTest = false;
		}
		if(rightJoyStick -> GetRawButton(2) == false){
			invertTest = true;
		}

		MotorR1->Set(throttlePre*invert + steerPre);
		MotorR2->Set(throttlePre*invert + steerPre);
		MotorL1->Set(-throttlePre*invert + steerPre);
		MotorL2->Set(-throttlePre*invert + steerPre);

		frc::Wait(0.005);
	}

	void TestPeriodic() {
	}

private:

};

START_ROBOT_CLASS(Robot)
