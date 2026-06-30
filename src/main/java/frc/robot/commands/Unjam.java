package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class Unjam extends Command {

    private Intake m_intake;
    private Hopper m_hopper;
    private Feeder m_feeder;
    private long startTime;

    public Unjam(Intake intake, Hopper hopper, Feeder feeder) {
        this.m_intake = intake;
        this.m_hopper = hopper;
        this.m_feeder = feeder;
        addRequirements(intake, hopper, feeder);
    }

    @Override
    public void initialize() {
        this.startTime = System.currentTimeMillis();
        if (m_intake.isClear()) {
            m_intake.setRPM(-2000);
            m_feeder.setOn();
            m_feeder.setBackward();
            m_hopper.setOn();
            m_hopper.setBackward();
        } else {
            m_intake.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 1000;
    }

    @Override
    public void end(boolean isFinished) {
        m_intake.stop();
        m_feeder.setOff();
        m_hopper.setOff();
        m_hopper.setForward();
        m_feeder.setForward();
    }
}
