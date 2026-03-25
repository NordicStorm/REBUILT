package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Shooter;


public class SetShooter extends InstantCommand implements CommandPathPiece{

    private Shooter m_Shooter;
    private Shooter.Mode mode;

    public SetShooter(Shooter shooter, Shooter.Mode mode) {
        this.m_Shooter = shooter;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        m_Shooter.setMode(mode);
    }
}
