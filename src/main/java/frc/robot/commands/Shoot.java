package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{

    private Shooter m_shooter;
    private Feeder m_feeder;
    private long startTime;

    public Shoot(Shooter shooter, Feeder feeder) {
        this.m_feeder = feeder;
        this.m_shooter = shooter;
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
        m_feeder.feed(0);
        m_shooter.shoot(0);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) > 1000;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.feed(0);
        m_shooter.shoot(0);
    }
}
