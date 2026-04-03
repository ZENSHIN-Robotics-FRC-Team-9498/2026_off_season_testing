package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;

import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase
{
    private static final String PID_KEY = "YAMS/Shooter_PID/";
    private static final double DEFAULT_P = 5;
    private static final double DEFAULT_I = 0.0;
    private static final double DEFAULT_D = 5;

    private final SparkMax flywheelMotor1 = new SparkMax(ShooterConstants.SHOOTER_1_CAN_ID, MotorType.kBrushless);
    private final SparkMax flywheelMotor2 = new SparkMax(ShooterConstants.SHOOTER_2_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig pidConfig = new SparkMaxConfig();
    private double lastP = DEFAULT_P;
    private double lastI = DEFAULT_I;
    private double lastD = DEFAULT_D;

    private final boolean flywheelMotor2Inverted = true;

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(DEFAULT_P, DEFAULT_I, DEFAULT_D)
        .withGearing(new MechanismGearing(1))
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0.43, 0.37, 0))
        .withFollowers(Pair.of(flywheelMotor2, flywheelMotor2Inverted))
        .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController motor = new SparkWrapper(flywheelMotor1, DCMotor.getNEO(2), motorConfig);
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motor)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(ShooterConstants.FLYWHEEL_DIAMETER_INCHES))
        // Mass of the flywheel.
        .withMass(Kilograms.of(ShooterConstants.FLYWHEEL_MASS_KG))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    public ShooterSubsystem() {
        SmartDashboard.putNumber(PID_KEY + "P", DEFAULT_P);
        SmartDashboard.putNumber(PID_KEY + "I", DEFAULT_I);
        SmartDashboard.putNumber(PID_KEY + "D", DEFAULT_D);
    }

    /**
     * Gets the current velocity of the shooter.
     *
     * @return FlyWheel velocity.
     */
    public AngularVelocity getVelocity() {return shooter.getSpeed();}

    /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {return shooter.set(dutyCycle);}


    public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

    public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.run(speed);}

    @Override
    public void simulationPeriodic()
    {
        shooter.simIterate();
    }

    @Override
    public void periodic()
    {
        if (!DriverStation.isFMSAttached()) {
            updatePidFromDashboard();
        }
        shooter.updateTelemetry();
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
        flywheelMotor1.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setRPM(LinearVelocity newHorizontalSpeed)
    {
        shooter.setMeasurementVelocitySetpoint(newHorizontalSpeed);
    }

    public boolean readyToShoot(AngularVelocity tolerance)
    {
        if (motor.getMechanismSetpointVelocity().isEmpty())
        {return false;}
        return motor.getMechanismVelocity().isNear(motor.getMechanismSetpointVelocity().orElseThrow(), tolerance);
    }

    public void setVelocitySetpoint(AngularVelocity speed)
    {
        shooter.setMechanismVelocitySetpoint(speed);
    }

    /**
     * Set speed from -1 to 1
     * @param speed Duty cycle to set the shooter to, from -1 to 1
     */
    public void setSpeed(double speed)
    {
        shooter.setDutyCycleSetpoint(speed);
    }

    public void stop() {setSpeed(0);}
}