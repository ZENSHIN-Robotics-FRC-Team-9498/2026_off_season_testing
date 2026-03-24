package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    @SuppressWarnings("unused")
    private final TalonFX leftMotor = new TalonFX(ClimberConstants.LEFT_MOTOR_CAN_ID);
    @SuppressWarnings("unused")
    private final TalonFX rightMotor = new TalonFX(ClimberConstants.RIGHT_MOTOR_CAN_ID);

    public ClimberSubsystem() {
        
    }

    public void climbToAngle(double angle) {

    }

    public void extend() {

    }

    public void retract() {
        
    }

    @Override
    public void periodic() {
        
    }
}
