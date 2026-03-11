package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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

    private final TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_CAN_ID);//, MotorType.kBrushless);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(-30), Degrees.of(100))
        .withGearing(new MechanismGearing(36))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
        .withMaxRobotHeight(Meters.of(1.5))
        .withMaxRobotLength(Meters.of(0.75))
        .withRelativePosition(new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0)));
        // TODO: fill in the above with actual measurements,
        // x = forward/back offset
        // y = left/right offset
        // z = height above robot origin
        // yaw = turret mounting rotation (usually 0)

    private final PivotConfig m_config = new PivotConfig(motor)
        .withHardLimit(Degrees.of(-100), Degrees.of(200))
        .withSoftLimits(Degrees.of(-10), Degrees.of(10))
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
    private final Transform3d roboToTurret = new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), Rotation3d.kZero);

    public TurretSubsystem()
    {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
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

    public void periodic()
    {
        turret.updateTelemetry();
    }

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
}