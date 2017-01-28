#include "WPILib.h"
#include "CANTalon.h"
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <CameraServer.h>
#include <thread>

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
	static void VisionThread()
		    {
		        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		        camera.SetResolution(640, 480);
		        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		        cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
		        cv::Mat source;
		        cv::Mat output;
		        while(true) {
		            cvSink.GrabFrame(source);
		            cvtColor(source, output, cv::COLOR_BGR2GRAY);
		            outputStreamStd.PutFrame(output);
		        }
		    }
public:

	   void RobotInit(){
	        std::thread visionThread(VisionThread);
	        visionThread.detach();
	    }

	Robot(){
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

			//Reverses motor direction
			if(rightJoyStick->GetRawButton(2)&invertTest)
			{
				invert *= -1;
				invertTest = false;
			}
			if(rightJoyStick->GetRawButton(2)==false){
				invertTest = true;
			}



			// Debug raw Joy-stick input to the driver station
			SmartDashboard::PutNumber("DB/Slider 0", leftJoyStick -> GetY());
			SmartDashboard::PutNumber("DB/Slider 1", rightJoyStick -> GetY());



			//Exponential function to give driver more control
			normalize(rightPower);
			normalize(leftPower);
			if(invert<0)
			{
			MotorL1->Set(-rightPower*invert);
			MotorL2->Set(-rightPower*invert);
			MotorR1->Set(leftPower*invert);
			MotorR2->Set(leftPower*invert);
			Wait(0.005);				// wait for a motor update time
			}
			else
			{
				MotorR1->Set(rightPower*invert);
				MotorR2->Set(rightPower*invert);
				MotorL1->Set(-leftPower*invert);
				MotorL2->Set(-leftPower*invert);
				Wait(0.005);
			}
		}
	}
};

START_ROBOT_CLASS(Robot)
