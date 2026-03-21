package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class doIntake extends Command {

    private Intake m_intake;
    private final boolean movingIntakeUp;

    public doIntake(Intake intake, boolean movingIntakeUp) {
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
    public void execute() {
        double intakeRPM = SmartDashboard.getNumber("Intake RPS Request", 0);
        if (!movingIntakeUp) {
            m_intake.setRPM(intakeRPM);
        } else {
            m_intake.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return m_intake.atSetPoint();
    }
}
