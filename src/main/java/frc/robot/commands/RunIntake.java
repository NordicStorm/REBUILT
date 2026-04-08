package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command implements CommandPathPiece {

    private Intake m_intake;
    private final boolean runIntake;
    private final boolean forward;

    public RunIntake(Intake intake, boolean runIntake, boolean forward) {
        this.m_intake = intake;
        this.runIntake = runIntake;
        this.forward = forward;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (runIntake && m_intake.isClear()) {
            m_intake.setRPM(forward ? 4000 : -2000);
        } else {
            m_intake.stop();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
