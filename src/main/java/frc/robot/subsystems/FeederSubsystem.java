package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.utils.SparkMAXContainer;

public class FeederSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_indexer = new SparkMAXContainer(ManipulatorConstants.INDEXER_CAN_ID);

    public FeederSubsystem() {
        m_indexer.setBreakMode(false);

        SmartDashboard.putNumber("Set feeder feed percent", 0);
        SmartDashboard.putNumber("Set feeder reject percent", 0);
    }

    public void feed() {
        m_indexer.motor.set(ManipulatorConstants.INDEXER_IN_SPEED);
    }

    public void reject() {
        m_indexer.motor.set(ManipulatorConstants.INDEXER_OUT_SPEED);
    }

    public void stop() {
        m_indexer.motor.stopMotor();
    }

    @Override
    public void periodic() {
    }
}