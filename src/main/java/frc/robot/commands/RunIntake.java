package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {

    private Intake m_intake;
    private final boolean runIntake;

    public RunIntake(Intake intake, boolean runIntake) {
        this.m_intake = intake;
        this.runIntake = runIntake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
       if (runIntake && m_intake.isClear()) {
            m_intake.setRPM(5400);
        } else {
            m_intake.stop();
        }
    }
}
