package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase
{
  private static final String PID_KEY = "YAMS/Hood_PID/";
  private static final double DEFAULT_P = 4.0;
  private static final double DEFAULT_I = 0.0;
  private static final double DEFAULT_D = 0.0;

  private double lastDis = 0;

  private final SparkMax hoodMotor= new SparkMax(ShooterConstants.ACTUATOR_CAN_ID, MotorType.kBrushless);
  private final SparkMaxConfig pidConfig = new SparkMaxConfig();
  private double lastP = DEFAULT_P;
  private double lastI = DEFAULT_I;
  private double lastD = DEFAULT_D;

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(DEFAULT_P, DEFAULT_I, DEFAULT_D, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(0), Degrees.of(45))
      .withGearing(new MechanismGearing(1))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new SparkWrapper(hoodMotor, DCMotor.getNEO(1), motorConfig);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0)));
      // TODO: fill in the above with actual measurements,
      // x = forward/back offset
      // y = left/right offset
      // z = height above robot origin

  private final PivotConfig m_config = new PivotConfig(motor)
      .withMOI(1) // affects sim only
      .withHardLimit(Degrees.of(0), Degrees.of(45))
      .withTelemetry("Hood", TelemetryVerbosity.HIGH)
      .withStartingPosition(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism);

  private final Pivot hood = new Pivot(m_config);


    public HoodSubsystem()
    {
        SmartDashboard.putNumber(PID_KEY + "P", DEFAULT_P);
        SmartDashboard.putNumber(PID_KEY + "I", DEFAULT_I);
        SmartDashboard.putNumber(PID_KEY + "D", DEFAULT_D);
        // temporary test control: set this to a non-zero percent on SmartDashboard to run the hood open-loop
        SmartDashboard.putNumber("HoodTestPercent", 0.0);
    }

    @Override
    public void periodic()
    {
        if (!DriverStation.isFMSAttached()) {
            updatePidFromDashboard();
        }
        hood.updateTelemetry();

        lastD = getDistanceMeters();
        double angleDeg = lastD * 70; // TODO: actually use linear regresion   

        hood.set(angleDeg).schedule();

        if(DriverStation.isFMSAttached()) {
            return;
        }
        
        SmartDashboard.putNumber("setpoint", this.hood.getMechanismSetpoint().orElse(Degrees.of(0)).baseUnitMagnitude());
        SmartDashboard.putNumber("distance", lastD);
        SmartDashboard.putNumber("angle", this.hood.getAngle().baseUnitMagnitude());

        // Read a dashboard value for quick manual open-loop testing. If non-zero, schedule a short-running
        // open-loop command to apply the percent to the hood motor. This is intentionally minimal and
        // diagnostic-only; set the value back to 0 to stop.
        // double testPct = SmartDashboard.getNumber("HoodTestPercent", 0.0);
            // schedule an open-loop command (the Pivot.set(...) call returns a Command)
            // hood.set(testPct).schedule();
        
    }

    public double getDistanceMeters() {
        var limelight = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry ty = limelight.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        if(limelight.getEntry("tv").getDouble(0.0) == 0.0) {
            return lastDis;
        }

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = Inches.convertFrom(615, Millimeters);

        // distance from the target to the floor
        double goalHeightInches = 44.25;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) * 25.4 / 1000;
    }
    
    @Override
    public void simulationPeriodic()
    {
        hood.simIterate();
    }

    public Command hoodCmd(double dutycycle)
    {
        return hood.set(dutycycle);
    }

    public Command setAngle(Angle angle)
    {
        return hood.setAngle(angle);
    }

    public void setAngleSetpoint(Angle angle)
    {
        hood.setMechanismPositionSetpoint(angle);
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
        hoodMotor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
