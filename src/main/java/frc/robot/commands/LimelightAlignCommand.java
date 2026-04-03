package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import static edu.wpi.first.units.Units.*;

public class LimelightAlignCommand extends Command {
    public static double speedFactor = .4;
    public static double rotationFactor = 0.4;
    public static DoubleSupplier MaxSpeed = () -> speedFactor * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static DoubleSupplier MaxAngularRate = () -> RotationsPerSecond.of(rotationFactor).in(RadiansPerSecond);

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed.getAsDouble() * OIConstants.kDriveDeadband).withRotationalDeadband(MaxAngularRate.getAsDouble() * OIConstants.kDriveDeadband) // Add deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final CommandSwerveDrivetrain swerve;
    private final CommandPS5Controller cont;
    private final IntakeSubsystem intake;

    private double lastDis = 0;

    private final PIDController turnPID;
    private final PIDController forwardPID;
    private final PIDController strafePID;

    private static final double TARGET_ROT = 0.0; // tx
    private static final double TARGET_FWD = 1.5; // meters, distance to target
    private static final double TARGET_LR  = 0.0; // centered on target

    private final Timer stabilityTimer = new Timer();
    private static final double HOLD_TIME = 0.25;

    public LimelightAlignCommand(CommandSwerveDrivetrain swerve, CommandPS5Controller cont, IntakeSubsystem intake) {
        this.swerve = swerve;
        this.cont = cont;
        this.intake = intake;

        turnPID = new PIDController(0.1, 0, 0.00);
        forwardPID = new PIDController(1.2, 0.0, 0.02);
        strafePID = new PIDController(1.2, 0.0, 0.02);

        turnPID.setTolerance(1.0);
        forwardPID.setTolerance(0.05);
        strafePID.setTolerance(0.05);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var limelight = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry ty = limelight.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = Inches.convertFrom(615, Millimeters);

        // distance from the target to the floor
        double goalHeightInches = 44.25;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        lastDis = (goalHeightInches - limelightLensHeightInches) / 
                                                    Math.tan(angleToGoalRadians);


        double tx = limelight.getEntry("tx").getDouble(0.0);

        double[] targetPose = limelight.getEntry("targetpose_robotspcae").getDoubleArray(new double[6]);
        double y = targetPose[0];

        double forwardSpeed = forwardPID.calculate(0, TARGET_FWD);
        double strafeSpeed  = strafePID.calculate(y, TARGET_LR);
        final double turnSpeed = turnPID.atSetpoint() ? 0.0 : turnPID.calculate(tx, TARGET_ROT);

        if (forwardPID.atSetpoint()) forwardSpeed = 0.0;
        if (strafePID.atSetpoint()) strafeSpeed = 0.0;


        swerve.applyRequest(() -> drive
            .withVelocityX(MathUtil.applyDeadband(cont.getLeftX(), OIConstants.kDriveDeadband))   // forward/back (meters/sec)
            .withVelocityY(-MathUtil.applyDeadband(cont.getLeftY(), OIConstants.kDriveDeadband))    // left/right (meters/sec)
            .withRotationalRate(turnSpeed) // radians/sec
        );

        if(forwardPID.atSetpoint() && strafePID.atSetpoint() && turnPID.atSetpoint()) {
            if(!stabilityTimer.isRunning()) {
                stabilityTimer.reset();
                stabilityTimer.start();
            }
        } else {
            stabilityTimer.stop();
            stabilityTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.applyRequest(() -> brake);
        stabilityTimer.stop();
        stabilityTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return stabilityTimer.hasElapsed(HOLD_TIME);
    }
}