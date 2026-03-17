package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class runHopper extends Command{

    private Hopper m_hopper;
    private long startTime;

    public runHopper(Hopper hopper) {
        this.m_hopper = hopper;
    }

    @Override
    public void initialize() {
        m_hopper.setRPM(0);
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double hopperRPM = SmartDashboard.getNumber("Hopper RPS Request", 0);
        m_hopper.setRPM(hopperRPM);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) > 2000;
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stop();
    }
}
