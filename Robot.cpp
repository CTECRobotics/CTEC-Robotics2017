#include <iostream>
#include <memory>
#include <string>
#include <stdlib.h>
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
#include <ADXRS450_Gyro.h>

class Robot: public frc::IterativeRobot {
	double VALVE_L;
	double VALVE_R;
	double VALVE_WAIT;
	double MASTER_TIME;
	//double VALVE_GEARBOX_L;
	//double VALVE_GEARBOX_R;
	Timer *AUTO_TIMER;
	CANTalon *MOTOR_LM;
	CANTalon *MOTOR_LS;
	CANTalon *MOTOR_RM;
	CANTalon *MOTOR_RS;
	CANTalon *MOTOR_SPINNER_ALPHA;
	CANTalon *MOTOR_SPINNER_BETA;
	PIDOutput *HORIZONTAL_MOTOR;
	Encoder *MOTOR_ENCODER_L;
	Encoder *MOTOR_ENCODER_R;
	Encoder *SPINNER_RPM;
	Joystick *LEFT_JOYSTICK;
	Joystick *RIGHT_JOYSTICK;
	Solenoid *LAUNCHER_L;
	Solenoid *LAUNCHER_R;
	DoubleSolenoid *GEARBOX_MAIN;
	PIDController *MOTOR_L_PID;
	PIDController *MOTOR_R_PID;
	PIDController *SPINNER;
	PIDController *HORIZONTAL_DRIVE;
	ADXRS450_Gyro *CONTROL_GYRO;
	AnalogInput *TURRET_CONTROLLER;
	double TURRET_POSITION;
	double CAMERA_ERROR;			//POSTIVE IS NEGATIVE, RIGHT IS POSITIVE
	double GYRO_SETPOINT;
	double THROTTLE;
	double STEER;
	double MAXIMUM_RPM;
	//bool CONTROL_STATE_1;
	//bool CONTROL_STATE_2;
	public:
	void RobotInit() {
		VALVE_L = 0;
		VALVE_R = 1;
		VALVE_WAIT = 0.25;

		MASTER_TIME = 0;

		AUTO_TIMER = new Timer();

		MOTOR_LM = new CANTalon(1);
		MOTOR_LS = new CANTalon(2);
		MOTOR_RM = new CANTalon(3);
		MOTOR_RS = new CANTalon(4);
		//MOTOR_SPINNER_ALPHA = new CANTalon(5);
		//MOTOR_SPINNER_BETA = new CANTalon(6);
		//HORIZONTAL_MOTOR = new CANTalon(7);

		MOTOR_LS->SetControlMode(CANSpeedController::kFollower);
		MOTOR_LS->Set(1);

		MOTOR_RS->SetControlMode(CANSpeedController::kFollower);
		MOTOR_RS->Set(3);

		//MOTOR_SPINNER_BETA->SetControlMode(CANSpeedController::kFollower);
		//MOTOR_SPINNER_BETA->Set(5);
/*
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
*/
		//SPINNER_RPM = new Encoder(4, 5, false, Encoder::EncodingType::k4X);
		//SPINNER_RPM->SetMaxPeriod(.1);
		//SPINNER_RPM->SetMinRate(10);
		//SPINNER_RPM->SetDistancePerPulse(5);
		//SPINNER_RPM->SetReverseDirection(false);
		//SPINNER_RPM->SetSamplesToAverage(7);

		//LAUNCHER_L = new Solenoid(VALVE_L);
		//LAUNCHER_R = new Solenoid(VALVE_R);
		//GEARBOX_MAIN = new DoubleSolenoid(4, 5, 6);

		LEFT_JOYSTICK = new Joystick(0);
		//RIGHT_JOYSTICK = new Joystick(1);

		//MOTOR_L_PID = new PIDController(1, 1, 1, MOTOR_ENCODER_L, MOTOR_LM);
		//MOTOR_R_PID = new PIDController(1, 1, 1, MOTOR_ENCODER_R, MOTOR_RM);
		//SPINNER = new PIDController(1, 1, 1, SPINNER_RPM, MOTOR_SPINNER_ALPHA);
		HORIZONTAL_DRIVE = new PIDController(1, 1, 1, TURRET_CONTROLLER, HORIZONTAL_MOTOR);

		CONTROL_GYRO = new ADXRS450_Gyro();

		//CAMERA_ERROR = 0;
		GYRO_SETPOINT = 0;

		THROTTLE = 0;
		STEER = 0;

		MAXIMUM_RPM = 511;

		//CONTROL_STATE_1 = false;
		//CONTROL_STATE_2 = false;

		MOTOR_LM->Set(0);
		MOTOR_RM->Set(0);

		CONTROL_GYRO->Calibrate();
	}
private:
	void AutonomousInit() override {
		//MOTOR_ENCODER_L->Reset();
		//MOTOR_ENCODER_R->Reset();

		//MOTOR_L_PID->Disable();
		//MOTOR_R_PID->Disable();

		AUTO_TIMER->Reset();
		AUTO_TIMER->Start();

		GYRO_SETPOINT = CONTROL_GYRO->GetAngle();
		MASTER_TIME = AUTO_TIMER->Get();

		//CONTROL_GYRO->Reset();

		//MOTOR_L_PID->SetSetpoint(MAXIMUM_RPM * 0.8);
		//MOTOR_R_PID->SetSetpoint(MAXIMUM_RPM * 0.8);
	}
	void AutonomousPeriodic() {
		if(AUTO_TIMER->Get() >= 14) {
			MOTOR_LM->Set(0.25);
			MOTOR_RM->Set(-0.28);
		} else if(AUTO_TIMER->Get() >= 1) {
			if(CONTROL_GYRO->GetAngle() <= (GYRO_SETPOINT-2)) {
				MOTOR_LM->Set(0.15);
				MOTOR_RM->Set(-0.08);
			} else if(CONTROL_GYRO->GetAngle() >= (GYRO_SETPOINT+2)) {
				MOTOR_LM->Set(0.05);
				MOTOR_RM->Set(-0.18);
			} else if(CONTROL_GYRO->GetAngle() > (GYRO_SETPOINT-2) || CONTROL_GYRO->GetAngle() < (GYRO_SETPOINT+2)) {
				MOTOR_LM->Set(0.25);
				MOTOR_RM->Set(-0.28);
			}
		} else if(AUTO_TIMER->Get() >= 0) {
			MOTOR_LM->Set(0);
			MOTOR_RM->Set(0);
		}
	}
	void TeleopInit() {
		AUTO_TIMER->Stop();
		//MOTOR_L_PID->Disable();
		//MOTOR_R_PID->Disable();
	}

	void TeleopPeriodic() {
		THROTTLE = LEFT_JOYSTICK->GetY();
		STEER = LEFT_JOYSTICK->GetX();

		MOTOR_LM->Set(THROTTLE+STEER);
		MOTOR_RM->Set(-THROTTLE+STEER);

		//if(LEFT_JOYSTICK->GetRawButton(10)) {

		// }

		//SmartDashboard::PutNumber("RPMS_L",  MOTOR_ENCODER_L->GetRate());
		//SmartDashboard::PutNumber("RPMS_R",  MOTOR_ENCODER_R->GetRate());
		SmartDashboard::PutNumber("CURRENT_GRYO", CONTROL_GYRO->GetAngle());

		frc::Wait(0.005);
	}
};

START_ROBOT_CLASS(Robot)
