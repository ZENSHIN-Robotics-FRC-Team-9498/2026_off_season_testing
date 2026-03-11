package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.utils.SparkMAXContainer;
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
    private final SparkMAXContainer m_intakeRoller = new SparkMAXContainer(IntakeConstants.INTAKE_ROLLER_CAN_ID);
    private final SparkMax m_actuatorMotor = new SparkMax(IntakeConstants.INTAKE_ACTUATOR_CAN_ID, SparkMax.MotorType.kBrushless);

    private double actuator_kP = 0.1;
    private double actuator_kI = 0;
    private double actuator_kD = 0;

    private double actuatorAngle = 0;
    
    private double slurpPercent;
    private double spitPercent;
  
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(actuator_kP, actuator_kI, actuator_kD, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(0), Degrees.of(90))
        //TODO: figure out actual gearing and fill in the below
        .withGearing(new MechanismGearing(3))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController motor = new SparkWrapper(m_actuatorMotor, DCMotor.getNEO(1), motorConfig);
    
    private ArmConfig m_config = new ArmConfig(motor)
        .withLength(Meters.of(0.135))
        .withHardLimit(Degrees.of(-100), Degrees.of(200))
        .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(1))
        .withStartingPosition(Degrees.of(0));

    private final Arm arm = new Arm(m_config);


    public IntakeSubsystem() {
        m_intakeRoller.setBreakMode(false);

        SmartDashboard.putNumber("Set intake actuator_kP", 0.1);
        SmartDashboard.putNumber("Set intake actuator_kI", 0);
        SmartDashboard.putNumber("Set intake actuator_kD", 0);
        
        SmartDashboard.putNumber("Set intake actuator degrees", 0);

        SmartDashboard.putNumber("Set slurp roller percent", 0);
        SmartDashboard.putNumber("Set spit roller percent", 0);
    }

    private void slurp() {
        // m_intakeRoller.motor.set(IntakeConstants.ROLLER_IN_SPEED);
        m_intakeRoller.motor.set(slurpPercent);
        extendIntake();
    }

    private void spit() {
        // m_intakeRoller.motor.set(IntakeConstants.ROLLER_OUT_SPEED);
        m_intakeRoller.motor.set(spitPercent);
    }

    public void runIntake(boolean trueForIn) {
        if(trueForIn)
            slurp();
        else
            spit();
    }

    public void extendIntake() {
        arm.setAngle(Degrees.of(actuatorAngle));
    }

    public void retractIntake() {
        arm.setAngle(Degrees.of(0));
    }

    public void stop() {
        m_intakeRoller.motor.stopMotor();
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();

        if(DriverStation.isFMSAttached()) {
            return;
        }
        
        SmartDashboard.putNumber("Real intake actuator degrees", arm.getAngle().in(Degrees));
        
        actuatorAngle = SmartDashboard.getNumber("Set intake actuator degrees", 0);

        slurpPercent = SmartDashboard.getNumber("Set slurp roller percent", 0);
        spitPercent = SmartDashboard.getNumber("Set spit roller percent", 0);
    }
}