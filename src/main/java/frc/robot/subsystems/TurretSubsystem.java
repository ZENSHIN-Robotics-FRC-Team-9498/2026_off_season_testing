package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TurretConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase
{
    private static final String PID_KEY = "YAMS/Turret_PID/";
    private static final double DEFAULT_P = 4.0;
    private static final double DEFAULT_I = 0.0;
    private static final double DEFAULT_D = 0.0;

    private final TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_CAN_ID);//, MotorType.kBrushless);
    private double lastP = DEFAULT_P;
    private double lastI = DEFAULT_I;
    private double lastD = DEFAULT_D;
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(DEFAULT_P, DEFAULT_I, DEFAULT_D, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(-180), Degrees.of(180))    // this is the real limit
        .withGearing(new MechanismGearing(36))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, 0.2, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
        .withMaxRobotHeight(Meters.of(1.5))
        .withMaxRobotLength(Meters.of(0.68))
        .withRelativePosition(new Translation3d(Meters.of(0.218), Meters.of(0), Meters.of(0.29)));
        // TODO: fill in the above with actual measurements,
        // x = forward/back offset
        // y = left/right offset
        // z = height above robot origin
        // yaw = turret mounting rotation (usually 0)

    private final PivotConfig m_config = new PivotConfig(motor)
        .withHardLimit(Degrees.of(-100), Degrees.of(200))   // this affect sim only
        // .withSoftLimits(Degrees.of(-45), Degrees.of(45))
        .withMOI(1) // affects sim only
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withStartingPosition(Degrees.of(0))
        .withMechanismPositionConfig(robotToMechanism);

    private final Pivot turret = new Pivot(m_config);

    // Robot to turret transform, from center of robot to turret.
    // TODO: fill this in with proper measurements.
    // x = forward/back offset
    // y = left/right offset
    // z = height above robot origin
    // yaw = turret mounting rotation (usually 0)
    private final Transform3d roboToTurret = new Transform3d(Meters.of(0.218), Meters.of(0), Meters.of(0.29), Rotation3d.kZero);

    public TurretSubsystem()
    {
        SmartDashboard.putNumber(PID_KEY + "P", DEFAULT_P);
        SmartDashboard.putNumber(PID_KEY + "I", DEFAULT_I);
        SmartDashboard.putNumber(PID_KEY + "D", DEFAULT_D);
    }

    public Pose2d getPose(Pose2d robotPose)
    {
        return robotPose.plus(new Transform2d(
            roboToTurret.getTranslation().toTranslation2d(), roboToTurret.getRotation().toRotation2d()));
    }

    public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle)
    {
        var robotAngleRads = robotAngle.in(Radians);
        double turretVelocityX =
            robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
            * (roboToTurret.getY() * Math.cos(robotAngleRads)
                - roboToTurret.getX() * Math.sin(robotAngleRads));
        double turretVelocityY =
            robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
            * (roboToTurret.getX() * Math.cos(robotAngleRads)
                - roboToTurret.getY() * Math.sin(robotAngleRads));

        return new ChassisSpeeds(turretVelocityX,
                                turretVelocityY,
                                robotVelocity.omegaRadiansPerSecond + motor.getMechanismVelocity().in(RadiansPerSecond));
    }

    @Override
    public void periodic()
    {
        if (!DriverStation.isFMSAttached()) {
            updatePidFromDashboard();
        }
        turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic()
    {
        turret.simIterate();
    }

    public Command turretCmd(double dutycycle)
    {
        return turret.set(dutycycle);
    }

    public Command setAngle(Angle angle)
    {
        return turret.setAngle(angle);
    }

    public void setAngleSetpoint(Angle measure)
    {
        turret.setMechanismPositionSetpoint(measure);
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
        turretMotor.getConfigurator().apply(new Slot0Configs().withKP(p).withKI(i).withKD(d));
    }
}