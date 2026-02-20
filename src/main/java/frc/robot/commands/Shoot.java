package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    }

    @Override
    public void initialize() {
        m_feeder.setRPM(0);
        m_shooter.setRPM(0);
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double shooterRPM = SmartDashboard.getNumber("Shooter RPS Request", 0);
        double feederRPM = SmartDashboard.getNumber("Feeder RPS Request", 0);
        m_feeder.setRPM(feederRPM);
        m_shooter.setRPM(shooterRPM);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) > 100000;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.stop();
    }
}
