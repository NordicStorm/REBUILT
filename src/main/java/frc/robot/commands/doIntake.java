package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class doIntake extends Command {

    private Intake m_intake;
    private long startTime;

    public doIntake(Intake intake) {
        this.m_intake = intake;
    }

    @Override
    public void initialize() {
        this.startTime = System.currentTimeMillis();
        m_intake.setIntakeDown();
    }

    @Override
    public void execute() {
        double intakeRPM = SmartDashboard.getNumber("Intake RPS Request", 0);
        m_intake.setRPM(intakeRPM);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) > 3000;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeDown();
        m_intake.stop();
    }
}
