package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class runHopper extends Command{

    private Hopper m_hopper;

    public runHopper(Hopper hopper) {
        this.m_hopper = hopper;
    }

    @Override
    public void initialize() {
        m_hopper.setOff();
    }

    @Override
    public void execute() {
        m_hopper.setOn();
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.setOff();
    }
}
