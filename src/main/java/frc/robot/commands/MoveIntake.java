package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Intake;

public class MoveIntake extends Command implements CommandPathPiece {

    private Intake m_intake;
    private final boolean movingIntakeUp;

    public MoveIntake(Intake intake, boolean movingIntakeUp) {
        this.m_intake = intake;
        this.movingIntakeUp = movingIntakeUp;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (movingIntakeUp) {
            m_intake.setIntakeUp();
        } else {
            m_intake.setIntakeDown();
        }
    }

   
    @Override
    public boolean isFinished() {
        return m_intake.atSetPoint();
    }
}
