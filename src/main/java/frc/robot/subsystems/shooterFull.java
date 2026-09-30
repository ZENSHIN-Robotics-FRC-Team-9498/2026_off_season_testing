package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterSettings;

public class shooterFull extends SubsystemBase {
    private final TalonFX shooterMotor1 = new TalonFX(39);
    private final TalonFX shooterMotor2 = new TalonFX(32);
    private final TalonFX shooterMoter5 = new TalonFX(15);
    private final SparkMax shooterMotor3 = new SparkMax(36, MotorType.kBrushless);
    private final SparkMax shooterMotor4 = new SparkMax(37, MotorType.kBrushless);
    private final ShooterSettings settings = new ShooterSettings();

    public Command on() {
        // 設定は射撃開始時に読み、射撃中のDashboard編集で出力を急変させない。
        return Commands.defer(() -> {
            double flywheelOutput = settings.flywheelOutput();
            double feedOutput = settings.feedOutput();
            return Commands.sequence(
                runOnce(() -> {
                    stop();
                    shooterMotor3.set(flywheelOutput);
                    shooterMotor4.set(-flywheelOutput);
                }),
                Commands.waitSeconds(1.0),
                runOnce(() -> {
                    shooterMotor1.set(-feedOutput);
                    shooterMotor2.set(feedOutput);
                    shooterMoter5.set(feedOutput);
                }),
                Commands.idle(this)
            );
        }, Set.of(this)).finallyDo(this::stop);
    }

    public Command off() {
        return runOnce(this::stop);
    }

    private void stop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        shooterMoter5.set(0);
        shooterMotor3.set(0);
        shooterMotor4.set(0);
    }
}
