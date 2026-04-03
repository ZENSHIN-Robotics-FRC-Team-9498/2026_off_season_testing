package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.utils.SparkMAXContainer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {
    private static final String PID_KEY = "YAMS/IntakeArm/PID/";
    private final SparkMAXContainer m_intakeRoller = new SparkMAXContainer(IntakeConstants.INTAKE_ROLLER_CAN_ID);
    private final SparkMax m_actuatorMotor_H = new SparkMax(IntakeConstants.INTAKE_ACTUATOR_H_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_actuatorMotor_L = new SparkMax(IntakeConstants.INTAKE_ACTUATOR_L_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig pidConfig = new SparkMaxConfig();

    private final boolean actuator_L_inverted = true;

    private double actuator_kP = 1;
    private double actuator_kI = 0;
    private double actuator_kD = 0;
    private double lastP = actuator_kP;
    private double lastI = actuator_kI;
    private double lastD = actuator_kD;

    private double actuatorAngle = 45;

    private final SmartMotorControllerConfig motorConfig_H = new SmartMotorControllerConfig(this)
        .withClosedLoopController(actuator_kP, actuator_kI, actuator_kD, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(0), Degrees.of(90))   // this is real limit
        //TODO: figure out actual gearing and fill in the below
        .withGearing(new MechanismGearing(GearBox.fromTeeth(45, 15)))
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("IntakeArmHMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorControllerConfig motorConfig_L = new SmartMotorControllerConfig(this)
        .withClosedLoopController(actuator_kP, actuator_kI, actuator_kD, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(0), Degrees.of(90))   // this is real limit
        //TODO: figure out actual gearing and fill in the below
        .withGearing(new MechanismGearing(GearBox.fromTeeth(45, 15)))
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("IntakeArmLMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(actuator_L_inverted)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
    
    private final SmartMotorController motor_H = new SparkWrapper(m_actuatorMotor_H, DCMotor.getNEO(1), motorConfig_H);
    private final SmartMotorController motor_L = new SparkWrapper(m_actuatorMotor_L, DCMotor.getNEO(1), motorConfig_L);
    
    private ArmConfig m_config_H = new ArmConfig(motor_H)
        .withLength(Meters.of(0.135))
        .withHardLimit(Degrees.of(-100), Degrees.of(200)) // affects sim only
        .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(1))
        .withStartingPosition(Degrees.of(0));

    private ArmConfig m_config_L = new ArmConfig(motor_L)
        .withLength(Meters.of(0.135))
        .withHardLimit(Degrees.of(-100), Degrees.of(200)) // affects sim only
        .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(1))
        .withStartingPosition(Degrees.of(0));

    private final Arm arm_H = new Arm(m_config_H);
    private final Arm arm_L = new Arm(m_config_L);


    public IntakeSubsystem() {
        m_intakeRoller.setBreakMode(false);

        SmartDashboard.putNumber("Set slurp roller percent", 0);
        SmartDashboard.putNumber("Set spit roller percent", 0);
        SmartDashboard.putNumber("Set intake actuator degrees", 0);
        SmartDashboard.putNumber(PID_KEY + "P", actuator_kP);
        SmartDashboard.putNumber(PID_KEY + "I", actuator_kI);
        SmartDashboard.putNumber(PID_KEY + "D", actuator_kD);
    }

    private void slurp() {
        m_intakeRoller.motor.set(IntakeConstants.ROLLER_IN_SPEED);
        
        extendIntake();
    }

    private void spit() {
        m_intakeRoller.motor.set(IntakeConstants.ROLLER_OUT_SPEED);
        
    }

    public void runIntake(boolean trueForIn) {
        if(trueForIn)
            slurp();
        else
            spit();
    }

    public void extendIntake() {
        arm_H.setAngle(Degrees.of(actuatorAngle));
        arm_L.setAngle(Degrees.of(actuatorAngle));
    }

    public void retractIntake() {
        arm_H.setAngle(Degrees.of(0));
        arm_L.setAngle(Degrees.of(0));
    }

    /**
     * True when either arm is past {@link IntakeConstants#INTAKE_EXTENDED_DRIVE_THRESHOLD_DEGREES}
     * (deployed enough to warrant limiting drivetrain speed).
     */
    public boolean isExtendedForDriveSlowdown() {
        double h = arm_H.getAngle().in(Degrees);
        double l = arm_L.getAngle().in(Degrees);
        double maxDeg = Math.max(h, l);
        return maxDeg > IntakeConstants.INTAKE_EXTENDED_DRIVE_THRESHOLD_DEGREES;
    }

    public void stop() {
        m_intakeRoller.motor.stopMotor();
    }

    @Override
    public void periodic() {
        arm_H.updateTelemetry();
        arm_L.updateTelemetry();

        if(DriverStation.isFMSAttached()) {
            return;
        }
        updatePidFromDashboard();
        
        SmartDashboard.putNumber("Real intake actuator degrees heavy", arm_H.getAngle().in(Degrees));
        SmartDashboard.putNumber("Real intake actuator degrees light", arm_L.getAngle().in(Degrees));
        
        actuatorAngle = SmartDashboard.getNumber("Set intake actuator degrees", 0);
    }

    private void updatePidFromDashboard() {
        double p = SmartDashboard.getNumber(PID_KEY + "P", lastP);
        double i = SmartDashboard.getNumber(PID_KEY + "I", lastI);
        double d = SmartDashboard.getNumber(PID_KEY + "D", lastD);
        if (p == lastP && i == lastI && d == lastD) {
            return;
        }
        lastP = p;
        lastI = i;
        lastD = d;
        pidConfig.closedLoop.p(p).i(i).d(d);
        m_actuatorMotor_H.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_actuatorMotor_L.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}