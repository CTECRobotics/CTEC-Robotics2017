#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include "WPILib.h"
#include "CANTalon.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "doublesolenoid.h"
#include "Timer.h"
#include <PIDSource.h>
#include <AnalogGyro.h>
#include <cmath>

class Robot: public frc::IterativeRobot {
	int VALVE_L;
	int VALVE_R;
	int VALVE_WAIT;
	Timer *AUTO_TIMER;
	CANTalon *MOTOR_LM;
	CANTalon *MOTOR_LS;
	CANTalon *MOTOR_RM;
	CANTalon *MOTOR_RS;
	Encoder *MOTOR_ENCODER_L;
	Encoder *MOTOR_ENCODER_R;
	Joystick *LEFT_JOYSTICK;
	Joystick *RIGHT_JOYSTICK;
	Solenoid *LAUNCHER_L;
	Solenoid *LAUNCHER_R;
	PIDController *MOTOR_L_PID;
	PIDController *MOTOR_R_PID;
	AnalogGyro *CONTROL_GYRO;
	int CAMERA_ERROR;			//POSTIVE IS NEGATIVE, RIGHT IS POSITIVE
	int GYRO_SETPOINT;
public:
	void RobotInit() {
		VALVE_L = 0;
		VALVE_R = 0;

		VALVE_WAIT = 0.25;

		AUTO_TIMER = new Timer();

		MOTOR_LM = new CANTalon(1);
		MOTOR_LS = new CANTalon(2);
		MOTOR_RM = new CANTalon(3);
		MOTOR_RS = new CANTalon(4);

		MOTOR_LS->SetControlMode(CANSpeedController::kFollower);
		MOTOR_LS->Set(1);

		MOTOR_RS->SetControlMode(CANSpeedController::kFollower);
		MOTOR_RS->Set(3);

		MOTOR_ENCODER_L = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
		MOTOR_ENCODER_L->SetMaxPeriod(.1);
		MOTOR_ENCODER_L->SetMinRate(10);
		MOTOR_ENCODER_L->SetDistancePerPulse(5);
		MOTOR_ENCODER_L->SetReverseDirection(false);
		MOTOR_ENCODER_L->SetSamplesToAverage(7);

		MOTOR_ENCODER_R = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
		MOTOR_ENCODER_R->SetMaxPeriod(.1);
		MOTOR_ENCODER_R->SetMinRate(10);
		MOTOR_ENCODER_R->SetDistancePerPulse(5);
		MOTOR_ENCODER_R->SetReverseDirection(false);
		MOTOR_ENCODER_R->SetSamplesToAverage(7);

		LAUNCHER_L = new Solenoid(VALVE_L);
		LAUNCHER_R = new Solenoid(VALVE_R);

		LEFT_JOYSTICK = new Joystick(1);
		RIGHT_JOYSTICK = new Joystick(2);

		MOTOR_LM->Set(0);
		MOTOR_RM->Set(0);

		MOTOR_L_PID = new PIDController(1, 1, 1, MOTOR_ENCODER_L, MOTOR_LM);
		MOTOR_R_PID = new PIDController(1, 1, 1, MOTOR_ENCODER_R, MOTOR_RM);

		CAMERA_ERROR = 0;
		GYRO_SETPOINT = 0;
	}
	void Robot::SHIFT_LOW() {

	}
	void Robot::SHIFT_HIGH() {

	}
	void Robot::BUCKET_LAUNCH() {
		LAUNCHER_L->Set(true);
		LAUNCHER_R->Set(true);
		Wait(VALVE_WAIT);
		LAUNCHER_L->Set(false);
		LAUNCHER_R->Set(false);
	}
	void Robot::ERROR_CORRECTION(CAMERA_ERROR) {

	}
	void AutonomousInit() override {
		MOTOR_L_PID->Enable();
		MOTOR_R_PID->Enable();

		AUTO_TIMER->Reset();
		AUTO_TIMER->Start();

		MOTOR_ENCODER_L->Reset();
		MOTOR_ENCODER_R->Reset();

		CONTROL_GYRO->Reset();
		GYRO_SETPOINT = CONTROL_GYRO->GetAngle();
	}

	void AutonomousPeriodic() {
		if(AUTO_TIMER->Get() >= 10) {
			//CONTROL IS WITH ENCODERS
			if(MOTOR_ENCODER_L->GetRate() > MOTOR_ENCODER_R->GetRate()) {

				MOTOR_L_PID->SetSetpoint(1);
				MOTOR_R_PID->SetSetpoint(2);
			} else if (MOTOR_ENCODER_L->GetRate() < MOTOR_ENCODER_R->GetRate()) {
				MOTOR_L_PID->SetSetpoint(2);
				MOTOR_R_PID->SetSetpoint(1);
			} else {
				MOTOR_L_PID->SetSetpoint(1);
				MOTOR_R_PID->SetSetpoint(1);
			}
		} else if (AUTO_TIMER->Get() >= 5) {
			//CONTROL HANDED OFF TO GYRO-CAMERA
			if(CAMERA_ERROR < 0) {

			} else if (CAMERA_ERROR > 0) {

			} else {

			}
		} else if (AUTO_TIMER->Get() >= 0) {
			//CONTROL IS HANDED TO FINAL APPROACH SEQUENCE
		}
	}

	void TeleopInit() {
		MOTOR_L_PID->Disable();
		MOTOR_R_PID->Disable();
	}

	void TeleopPeriodic() {

	}

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
