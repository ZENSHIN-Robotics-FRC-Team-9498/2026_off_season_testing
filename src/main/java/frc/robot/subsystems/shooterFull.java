package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterFull extends SubsystemBase {
    private final TalonFX shooterMotor1 = new TalonFX(39);
    private final TalonFX shooterMotor2 = new TalonFX(32);
    private final TalonFX shooterMoter5 = new TalonFX(15);
    private final SparkMax shooterMotor3 = new SparkMax(36, MotorType.kBrushless);
    private final SparkMax shooterMotor4 = new SparkMax(37, MotorType.kBrushless);

    /** Creates a new ExampleSubsystem. */
    public shooterFull() {
    }

    public Command on() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
            () -> {
            shooterMotor3.set(1);
            shooterMotor4.set(-1);
            try {
                Thread.sleep(1000); 
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            shooterMotor1.set(-0.5);
            shooterMotor2.set(0.5);
            shooterMoter5.set(0.5);
            });
    }

    public Command off() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
            shooterMotor3.set(0);
            shooterMotor4.set(0);
            shooterMoter5.set(0);
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}