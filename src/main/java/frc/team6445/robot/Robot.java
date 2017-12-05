package frc.team6445.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Robot extends IterativeRobot {
    NetworkTable Table = NetworkTable.getTable("SmartDashboard");
    private double VALVE_L;
    private double VALVE_R;
    private double MASTER_TIME;
    //CONTROL VALUES
    private double TURRET_SETPOINT;
    private double CAMERA_ERROR;	//POSTIVE IS NEGATIVE, RIGHT IS POSITIVE
    private double GYRO_SETPOINT;
    private double CURRENT_SETPOINT;
    private double DISTANCE;
    private double RANGE_METERS;
    private double OUTPUT;
    //MODIFIABLE VALUES
    private double THROTTLE;
    private double STEER;
    //DON'T TOUCH ME VALUES
    private double MAXIMUM_RPM;
    private double VALVE_WAIT;
    //MISC
    //double VALVE_GEARBOX_L;
    //double VALVE_GEARBOX_R;
    //CANTALON OBJECTS
    private CANTalon MOTOR_LM;
    private CANTalon MOTOR_LS;
    private CANTalon MOTOR_RM;
    private CANTalon MOTOR_RS;
    private CANTalon MOTOR_SPINNER_ALPHA;
    private CANTalon MOTOR_SPINNER_BETA;
    // PID STUFF
    private PIDOutput HORIZONTAL_MOTOR;
    private PIDController MOTOR_L_PID;
    private PIDController MOTOR_R_PID;
    private PIDController SPINNER;
    private PIDController HORIZONTAL_DRIVE;
    //SOLENOIDS
    private Solenoid LAUNCHER_L;
    private Solenoid LAUNCHER_R;
    private DoubleSolenoid GEARBOX_MAIN;
    // SENSOR INPUTS
    private Encoder MOTOR_ENCODER_L;
    private Encoder MOTOR_ENCODER_R;
    private Encoder SPINNER_RPM;
    private ADXRS450_Gyro CONTROL_GYRO;
    private Potentiometer TURRET_CONTROLLER;
    //MISC
    private Timer AUTO_TIMER;
    private Joystick LEFT_JOYSTICK;
    private Joystick RIGHT_JOYSTICK;
    private AnalogInput AUTO_RANGEFINDER;
    public void robotInit() {
        NetworkTable.setClientMode();
        NetworkTable.setIPAddress("roborio-6445-frc.local");
        Table.putString("IS INIT?","ROBOT INITIALIZED");
        VALVE_L = 0;
        VALVE_R = 1;
        VALVE_WAIT = 0.25;
        DISTANCE = 0;
        OUTPUT = 0;
        TURRET_SETPOINT = 0;
        CURRENT_SETPOINT = 0;
        MASTER_TIME = 0;
        AUTO_TIMER = new Timer();
        MOTOR_LM = new CANTalon(1);
        MOTOR_LS = new CANTalon(2);
        MOTOR_RM = new CANTalon(3);
        MOTOR_RS = new CANTalon(4);
        //MOTOR_SPINNER_ALPHA = new CANTalon(5);
        //MOTOR_SPINNER_BETA = new CANTalon(6);
        HORIZONTAL_MOTOR = new CANTalon(7);
        TURRET_CONTROLLER = new AnalogPotentiometer(1, 3600, 1800);
        MOTOR_LS.changeControlMode(CANTalon.TalonControlMode.Follower);
        MOTOR_LS.set(1);
        MOTOR_RS.set(3);
        //MOTOR_SPINNER_BETA.changeControlMode(CANTalon.TalonControlMode.Follower);
        //MOTOR_SPINNER_BETA.Set(5);
        MOTOR_ENCODER_L = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
        MOTOR_ENCODER_L.setMaxPeriod(.1);
        MOTOR_ENCODER_L.setMinRate(10);
        MOTOR_ENCODER_L.setDistancePerPulse(5);
        MOTOR_ENCODER_L.setReverseDirection(false);
        MOTOR_ENCODER_L.setSamplesToAverage(7);

        MOTOR_ENCODER_R = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
        MOTOR_ENCODER_R.setMaxPeriod(.1);
        MOTOR_ENCODER_R.setMinRate(10);
        MOTOR_ENCODER_R.setDistancePerPulse(5);
        MOTOR_ENCODER_R.setReverseDirection(false);
        MOTOR_ENCODER_R.setSamplesToAverage(7);
        //SPINNER_RPM = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
        //SPINNER_RPM.SetMaxPeriod(.1);
        //SPINNER_RPM.SetMinRate(10);
        //SPINNER_RPM.SetDistancePerPulse(5);
        //SPINNER_RPM.SetReverseDirection(false);
        //SPINNER_RPM.SetSamplesToAverage(7);

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

        MOTOR_LM.set(0);
        MOTOR_RM.set(0);

        CONTROL_GYRO.calibrate();

        AUTO_RANGEFINDER = new AnalogInput(0);
    }
    double VOLTAGE_CONVERT(double INPUT_VOLTAGE){
        RANGE_METERS = (INPUT_VOLTAGE*5000)/4.88;
        return RANGE_METERS;
    }
    public void disabledInit() {
    Table.putString("STATUS", "ROBOT IS DISABLED :(");
    }

    public void autonomousInit() {
    HORIZONTAL_DRIVE.disable();
    AUTO_TIMER.reset();
    AUTO_TIMER.start();
    GYRO_SETPOINT = CONTROL_GYRO.getAngle();
    MASTER_TIME = AUTO_TIMER.get();
    }

    public void teleopInit() {
    AUTO_TIMER.stop();
    HORIZONTAL_DRIVE.enable();
    TURRET_SETPOINT =  TURRET_CONTROLLER.get();
    CURRENT_SETPOINT = TURRET_SETPOINT;
    }

    public void disabledPeriodic() {
    Table.putString("STATUS", "ROBOT IS DISABLED!");
    }

    public void autonomousPeriodic() {
        Table.putString("STATUS", "AUTONOMOUS ENABLED");
        if (AUTO_TIMER.get() >= 14) {
            MOTOR_LM.set(0.25);
            MOTOR_RM.set(-0.28);
        } else if(AUTO_TIMER.get() >=1){
            if(CONTROL_GYRO.getAngle() <= (GYRO_SETPOINT-2)){
                MOTOR_LM.set(0.15);
                MOTOR_RM.set(-0.08);
            } else if(CONTROL_GYRO.getAngle() >= (GYRO_SETPOINT+2)){
                MOTOR_LM.set(0.05);
                MOTOR_RM.set(-0.18);
            } else if(CONTROL_GYRO.getAngle() > (GYRO_SETPOINT-2) || CONTROL_GYRO.getAngle() < (GYRO_SETPOINT+2)){
                MOTOR_LM.set(0.25);
                MOTOR_RM.set(-0.28);
            } else if(AUTO_TIMER.get() >=0){
                MOTOR_LM.set(0);
                MOTOR_RM.set(0);

            }
        }
    }

    public void teleopPeriodic() {
        THROTTLE = LEFT_JOYSTICK.getY();
        STEER = LEFT_JOYSTICK.getX();
        MOTOR_LM.set(THROTTLE+STEER);
        MOTOR_RM.set(-THROTTLE+STEER);
        if(LEFT_JOYSTICK.getZ() > 0.25) {
            HORIZONTAL_DRIVE.setSetpoint(CURRENT_SETPOINT + LEFT_JOYSTICK.getZ());
        } else if(LEFT_JOYSTICK.getZ() < -0.25) {
            HORIZONTAL_DRIVE.setSetpoint(CURRENT_SETPOINT - LEFT_JOYSTICK.getZ());
        } else {
            HORIZONTAL_DRIVE.setSetpoint(CURRENT_SETPOINT);
        }

        OUTPUT = VOLTAGE_CONVERT(AUTO_RANGEFINDER.getVoltage());


        Table.putValue("CURRENT_GRYO", CONTROL_GYRO.getAngle());
        //SmartDashboard::PutNumber("CURRENT POSTION", TURRET_CONTROLLER->Get());
        Table.putValue("RANGE", AUTO_RANGEFINDER.getVoltage());
        Table.putValue("RANGE_METERS", OUTPUT);

        Timer.delay(0.005);
    }
}
