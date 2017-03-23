bool autonomousFlag = true;

void Robot::AutonomousInit(){
    autonomousFlag = true;
}

void Robot::Autonomous(){

    if(currentTime < 5){
        // Drive Forward
        MotorR2->set(forwardVelocityAuto);
        MotorL2->Set(-forwardVelocityAuto);
    }   else{
        if(autonomousFlag){
            autonomousFlag = false;
            gearSetPoint = positionDown;
            // drop gearController
        }
    }

	gearController->SetPID(SmartDashboard::GetNumber("P", 0),SmartDashboard::GetNumber("I", 0),SmartDashboard::GetNumber("D", 0));
	gearController->SetOutputRange(-0.2, 0.1);
	gearController->SetAbsoluteTolerance(.01);
	gearController->SetSetpoint(gearSetPoint);
}

Robot::~Robot(){
    delete MotorR1;
    delete MotorR2;
    delete MotorR3;
    delete MotorL1;
    delete MotorL2;
    delete MotorL3;
}