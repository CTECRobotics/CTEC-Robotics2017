#include "WPILib.h"
#include "CANTalon.h"
#include <cmath>
#include "CameraServer.h"

//TANK DRIVE
/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot{
private:

	Joystick *leftJoyStick;
	Joystick *rightJoyStick;

	CANTalon *MotorR1;
	CANTalon *MotorR2;
	CANTalon *MotorL1;
	CANTalon *MotorL2;
	double thresh;
	int invert;
	bool invertTest;

public:
	void normalize(double &input){
		if (input > 0){
			input =  (std::pow(2, input) - 1);
		}else{
			input =  -(std::pow(2, std::abs(input)) - 1);
		}
	}

	Robot(){
		//cs::UsbCamera StartAutomaticCapture("Cam0", "/dev/video0");
		//CameraServer::SetSize(kSize640x480);
		this -> invert = 1;
		this -> thresh = .1;
		this -> invertTest = true;

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
	void OperatorControl()
	{
		while (true)
		{
			double leftPower = leftJoyStick->GetY();
			double rightPower = rightJoyStick->GetY();

			// Debug raw Joy-stick input to the driver station
			SmartDashboard::PutNumber("DB/Slider 0", leftJoyStick -> GetY());
			SmartDashboard::PutNumber("DB/Slider 1", rightJoyStick -> GetY());


			SmartDashboard::PutNumber("DB/Slider 2", invert);
			SmartDashboard::PutNumber("DB/Slider 3", invertTest);

			//Exponential function to give driver more control
			normalize(rightPower);
			normalize(leftPower);

			MotorR1->Set(rightPower*invert);
			MotorR2->Set(rightPower*invert);
			MotorL1->Set(-leftPower*invert);
			MotorL2->Set(-leftPower*invert);
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(Robot)
