/*
 * Arcade.h
 *
 *  Created on: Feb 19, 2017
 *      Author: ImMilesAhead
 */

#ifndef SRC_ARCADE_H_
#define SRC_ARCADE_H_

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
#include "AnalogInput.h"

class Robot: public frc::IterativeRobot {
private:



	const double EPSILON = 1E-1;
	bool isInverted;	//True if robot drive is Inverted, false if not inverted
	double soloWait;	// Time it takes for the Solenoid to shift gears
	DoubleSolenoid *gearbox1;

	std::function<void()> autonomousProgram;
	AnalogInput potPort;
	PIDController *gearController;


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
public:
	void RobotInit();
	void shiftHigh();
	void shiftLow();
	void autonomousInit() override;
	void auton0();
	void auton1();
	void auton2();
	void auton3();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
}


#endif /* SRC_ARCADE_H_ */
