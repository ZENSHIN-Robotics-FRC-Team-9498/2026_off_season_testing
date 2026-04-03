package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.utils.TalonFxContainer;

/**
 * Flip climber: two Kraken X60s via {@link TalonFxContainer}, position setpoints in mechanism output
 * rotations (see {@link ClimberConstants#TOTAL_REDUCTION}).
 *
 * <p>TODO: Calibrate {@link ClimberConstants#RETRACT_POSITION_ROTATIONS} and {@link ClimberConstants#EXTEND_POSITION_ROTATIONS}
 * (and dashboard copies) on the real robot; tune {@code Climber/PID/*} and inversions.
 */
public class ClimberSubsystem extends SubsystemBase {
    private static final String PID_KEY = "Climber/PID/";

    private final TalonFxContainer m_left = new TalonFxContainer(ClimberConstants.LEFT_MOTOR_CAN_ID);
    private final TalonFxContainer m_right = new TalonFxContainer(ClimberConstants.RIGHT_MOTOR_CAN_ID);

    private double extendPositionRot = ClimberConstants.EXTEND_POSITION_ROTATIONS;
    private double retractPositionRot = ClimberConstants.RETRACT_POSITION_ROTATIONS;

    private double lastP = ClimberConstants.kP;
    private double lastI = ClimberConstants.kI;
    private double lastD = ClimberConstants.kD;
    private double lastS = ClimberConstants.kS;
    private double lastV = ClimberConstants.kV;
    private double lastG = ClimberConstants.kG;

    private double targetMechanismRotations = retractPositionRot;

    public ClimberSubsystem() {
        m_left.setBreakMode(true);
        m_right.setBreakMode(true);
        m_left.setGearRatio(ClimberConstants.TOTAL_REDUCTION);
        m_right.setGearRatio(ClimberConstants.TOTAL_REDUCTION);
        m_left.setCurrentLimit(ClimberConstants.CURRENT_LIMIT_AMPS);
        m_right.setCurrentLimit(ClimberConstants.CURRENT_LIMIT_AMPS);

        MotorOutputConfigs leftOut =
            ClimberConstants.invertLeftMotor
                ? new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                : new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        MotorOutputConfigs rightOut =
            ClimberConstants.invertRightMotor
                ? new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                : new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        m_left.applyMotorOutput(leftOut);
        m_right.applyMotorOutput(rightOut);

        Slot0Configs slot0 = buildSlot0();
        m_left.applySlot0(slot0);
        m_right.applySlot0(slot0);

        publishPidKeys();
        SmartDashboard.putNumber("Climber/Set Extend Position (rot)", extendPositionRot);
        SmartDashboard.putNumber("Climber/Set Retract Position (rot)", retractPositionRot);
    }

    private Slot0Configs buildSlot0() {
        return new Slot0Configs()
            .withKP(lastP)
            .withKI(lastI)
            .withKD(lastD)
            .withKS(lastS)
            .withKV(lastV)
            .withKG(lastG)
            // Flip arm: kG uses cosine of mechanism angle; set sensor 0 = horizontal per CTRE docs if using kG.
            .withGravityType(GravityTypeValue.Arm_Cosine);
    }

    private void publishPidKeys() {
        SmartDashboard.putNumber(PID_KEY + "P", lastP);
        SmartDashboard.putNumber(PID_KEY + "I", lastI);
        SmartDashboard.putNumber(PID_KEY + "D", lastD);
        SmartDashboard.putNumber(PID_KEY + "S", lastS);
        SmartDashboard.putNumber(PID_KEY + "V", lastV);
        SmartDashboard.putNumber(PID_KEY + "G", lastG);
    }

    /** Command both motors to the same mechanism position (output rotations). */
    public void setMechanismPosition(double mechanismRotations) {
        targetMechanismRotations = mechanismRotations;
        m_left.setMechanismPosition(mechanismRotations);
        m_right.setMechanismPosition(mechanismRotations);
    }

    /** Move to the configured extend setpoint (dashboard-tunable when not on FMS). */
    public void goToExtend() {
        setMechanismPosition(extendPositionRot);
    }

    /** Move to the configured retract setpoint (dashboard-tunable when not on FMS). */
    public void goToRetract() {
        setMechanismPosition(retractPositionRot);
    }

    /** @deprecated use {@link #setMechanismPosition(double)} */
    @Deprecated
    public void climbToAngle(double angle) {
        setMechanismPosition(angle);
    }

    public void stop() {
        m_left.stopMotor();
        m_right.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Left Position (rot)", m_left.motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Right Position (rot)", m_right.motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Left Velocity (rps)", m_left.motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Right Velocity (rps)", m_right.motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Left Current (A)", m_left.motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Right Current (A)", m_right.motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Configured Total Reduction", ClimberConstants.TOTAL_REDUCTION);
        SmartDashboard.putNumber("Climber/Target (rot)", targetMechanismRotations);

        if (DriverStation.isFMSAttached()) {
            return;
        }

        extendPositionRot = SmartDashboard.getNumber("Climber/Set Extend Position (rot)", extendPositionRot);
        retractPositionRot = SmartDashboard.getNumber("Climber/Set Retract Position (rot)", retractPositionRot);

        updatePidFromDashboard();
    }

    private void updatePidFromDashboard() {
        double p = SmartDashboard.getNumber(PID_KEY + "P", lastP);
        double i = SmartDashboard.getNumber(PID_KEY + "I", lastI);
        double d = SmartDashboard.getNumber(PID_KEY + "D", lastD);
        double s = SmartDashboard.getNumber(PID_KEY + "S", lastS);
        double v = SmartDashboard.getNumber(PID_KEY + "V", lastV);
        double g = SmartDashboard.getNumber(PID_KEY + "G", lastG);
        if (p == lastP && i == lastI && d == lastD && s == lastS && v == lastV && g == lastG) {
            return;
        }
        lastP = p;
        lastI = i;
        lastD = d;
        lastS = s;
        lastV = v;
        lastG = g;
        var slot =
            new Slot0Configs()
                .withKP(p)
                .withKI(i)
                .withKD(d)
                .withKS(s)
                .withKV(v)
                .withKG(g)
                .withGravityType(GravityTypeValue.Arm_Cosine);
        m_left.applySlot0(slot);
        m_right.applySlot0(slot);
    }
}
