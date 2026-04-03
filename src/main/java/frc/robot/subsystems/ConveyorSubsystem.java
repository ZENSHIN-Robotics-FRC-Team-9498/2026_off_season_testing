package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.utils.TalonFxContainer;

/**
 * Do not use directly. Access from the shooter instead
 */
public class ConveyorSubsystem extends SubsystemBase {
    private final TalonFxContainer m_feederBelt = new TalonFxContainer(ManipulatorConstants.CONVEYOR_CAN_ID);

    private double inPercent = ManipulatorConstants.CONVEYOR_IN_SPEED;
    private double outPercent = ManipulatorConstants.CONVEYOR_OUT_SPEED;

    public ConveyorSubsystem() {
        m_feederBelt.setBreakMode(false);

        SmartDashboard.putNumber("Set conveyer in percent", 0);
        SmartDashboard.putNumber("Set conveyer out percent", 0);
    }

    public void runConveyor() {
        // m_feederBelt.motor.set(ManipulatorConstants.CONVEYOR_IN_SPEED);
        m_feederBelt.motor.set(inPercent);
    }

    public void backfeedConveyor() {
        // m_feederBelt.motor.set(ManipulatorConstants.CONVEYOR_OUT_SPEED);
        m_feederBelt.motor.set(outPercent);
    }

    public void stop() {
        m_feederBelt.motor.stopMotor();
    }

    @Override
    public void periodic() {
        if(DriverStation.isFMSAttached()) {
            return;
        }
        
        inPercent = SmartDashboard.getNumber("Set conveyer in percent", 0);
        outPercent = SmartDashboard.getNumber("Set conveyer out percent", 0);
    }
}
