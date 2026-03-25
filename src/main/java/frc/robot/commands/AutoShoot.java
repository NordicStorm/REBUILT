package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Mode;
import frc.robot.subsystems.Hopper;

public class AutoShoot extends Command implements CommandPathPiece{

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Hopper m_hopper;
    private CommandSwerveDrivetrain m_drivetrain;
    private boolean isHub;
    private long startTime;

    public AutoShoot(Shooter shooter, Feeder feeder, Hopper hopper, CommandSwerveDrivetrain drivetrain, boolean isHub) {
        this.m_feeder = feeder;
        this.m_shooter = shooter;
        this.m_hopper = hopper;
        this.m_drivetrain = drivetrain;
        this.isHub = isHub;
    }

    @Override
    public void initialize() {
        m_feeder.setRPM(0);
        m_hopper.setRPM(0);
        if (isHub) {
            m_shooter.setMode(Mode.HUB);
        } else {
            m_shooter.setMode(Mode.PASS);
        }
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double shooterRPS = SmartDashboard.getNumber("Shooter RPS Request", 0);
        double feederRPM = SmartDashboard.getNumber("Feeder RPS Request", 0);
        int hoodAngle = (int) SmartDashboard.getNumber("Hood Pulse Request", 1100);
        SmartDashboard.putString("Curve Point", m_drivetrain.getDistanceToVirtualHub() + "," + shooterRPS + "," + hoodAngle);
        m_shooter.setManualRPS(shooterRPS);
        m_shooter.setHoodAngle(hoodAngle);
        if (m_shooter.atSetPoint()) {
            m_hopper.setRPM(feederRPM);
            m_feeder.setRPM(shooterRPS);
        } else {
            m_hopper.setRPM(0);
            m_feeder.setRPM(0);
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
