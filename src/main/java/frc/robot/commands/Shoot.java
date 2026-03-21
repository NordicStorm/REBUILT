package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;

public class Shoot extends Command{

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Hopper m_hopper;
    private long startTime;

    public Shoot(Shooter shooter, Feeder feeder, Hopper hopper) {
        this.m_feeder = feeder;
        this.m_shooter = shooter;
        this.m_hopper = hopper;
    }

    @Override
    public void initialize() {
        m_feeder.setRPM(0);
        m_shooter.setRPM(0);
        m_hopper.setRPM(0);
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double shooterRPM = SmartDashboard.getNumber("Shooter RPS Request", 0);
        double feederRPM = SmartDashboard.getNumber("Feeder RPS Request", 0);
        m_shooter.setRPM(shooterRPM);
        if (m_shooter.atSetPoint()) {
            m_hopper.setRPM(feederRPM);
            m_feeder.setRPM(shooterRPM);
        }
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) > 100000;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.stop();
    }

    public static double getRotationVelocity() {
        double current = RobotContainer.drivetrain.getGyroDegrees();
        double target = getAngleToTarget();
        return (current - target) * 0.1;
    }

    public static double getAngleToTarget() {
        Pose2d target = RobotContainer.targetPosition;
        Pose2d current = RobotContainer.drivetrain.getPose();
        return Math.toDegrees(Util.angleBetweenPoses(target, current));
    }
}
