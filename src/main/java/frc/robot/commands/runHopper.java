package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class runHopper extends Command{

    private Hopper m_hopper;

    public runHopper(Hopper hopper) {
        this.m_hopper = hopper;
    }

    @Override
    public void initialize() {
        m_hopper.setRPM(0);
    }

    @Override
    public void execute() {
        double hopperRPM = SmartDashboard.getNumber("Hopper RPS Request", -15);
        m_hopper.setRPM(hopperRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stop();
    }
}
