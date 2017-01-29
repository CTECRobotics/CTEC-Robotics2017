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
	double StartTime;
	int invert;
	bool invertTest;

	void normalize(double &input){
		if (input > 0){
			input =  (std::pow(2, input) - 1);
		}else{
			input =  -(std::pow(2, std::abs(input)) - 1);
		}
	}
	void RobotInit() {
		this -> invert = 1;
		this -> thresh = .1;
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
		double leftPower = leftJoyStick->GetY();
		double rightPower = rightJoyStick->GetY();

		//Reverses motor direction
		if(rightJoyStick->GetRawButton(2)&invertTest) {
			invert *= -1;
			invertTest = false;
		}
		if(rightJoyStick->GetRawButton(2)==false) {
			invertTest = true;
		}

		//Exponential function to give driver more control
		normalize(rightPower);
		normalize(leftPower);
		if(invert<0) {
		MotorL1->Set(-rightPower*invert);
		MotorL2->Set(-rightPower*invert);
		MotorR1->Set(leftPower*invert);
		MotorR2->Set(leftPower*invert);
		Wait(0.005);
		// wait for a motor update time
		} else {
		MotorR1->Set(rightPower*invert);
		MotorR2->Set(rightPower*invert);
		MotorL1->Set(-leftPower*invert);
		MotorL2->Set(-leftPower*invert);
		Wait(0.005);
		}
	}

	void TestPeriodic() {
	}

private:

};

START_ROBOT_CLASS(Robot)
