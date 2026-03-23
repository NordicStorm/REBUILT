package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetShooter extends InstantCommand {

    private Shooter m_Shooter;
    private boolean turnOn;

    public SetShooter(Shooter shooter, boolean turnOn) {
        this.m_Shooter = shooter;
        this.turnOn = turnOn;
    }

    @Override
    public void initialize() {
        if (turnOn) {
            
        }
    }
}
