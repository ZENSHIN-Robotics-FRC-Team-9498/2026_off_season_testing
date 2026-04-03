package frc.robot.commands;


import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireCommand extends Command {
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyer;
    private final ShooterSubsystem m_shooter;

    public FireCommand(FeederSubsystem feeder, ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        this.m_feeder = feeder;
        this.m_conveyer = conveyor;
        this.m_shooter = shooter;
        addRequirements(m_feeder, m_conveyer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooter.set(0.7);
        // m_shooter.setVelocitySetpoint(RPM.of(5000));
        m_feeder.feed();
        m_conveyer.runConveyor();
    }

    @Override
    public void end(boolean interrupted) {
        // m_shooter.setVelocitySetpoint(RPM.of(0));
        m_shooter.stop();
        m_feeder.stop();
        m_conveyer.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
