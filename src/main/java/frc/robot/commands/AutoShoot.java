package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
import frc.robot.subsystems.Intake;

public class AutoShoot extends Command implements CommandPathPiece {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Hopper m_hopper;
    private Intake m_intake;
    private CommandSwerveDrivetrain m_drivetrain;
    private boolean isHub;
    private long timeToRun;
    private long endTime;
    private ProfiledPIDController PID;

    public AutoShoot(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, CommandSwerveDrivetrain drivetrain, boolean isHub,
            long timeToRun) {
        this.m_feeder = feeder;
        this.m_shooter = shooter;
        this.m_hopper = hopper;
        this.m_intake = intake;
        this.m_drivetrain = drivetrain;
        this.isHub = isHub;
        this.timeToRun = timeToRun;
        PID = new ProfiledPIDController(5, 0, .5, new Constraints(8, 7));
        PID.enableContinuousInput(-Math.PI, Math.PI);
        PID.setTolerance(Math.toRadians(3));
        addRequirements(this.m_shooter, m_feeder, m_hopper);
    }

    @Override
    public void initialize() {

        m_feeder.setOff();
        m_hopper.setOff();
        PID.reset(m_drivetrain.getGyroRadians());

        this.endTime = System.currentTimeMillis() + timeToRun;
        m_intake.setAgitateMode(true);
    }

    @Override
    public void execute() {
        double shooterRPS = SmartDashboard.getNumber("Shooter RPS Request", -55);
        int hoodAngle = (int) SmartDashboard.getNumber("Hood Pulse Request", 1430);
        SmartDashboard.putString("Curve Point",
                m_drivetrain.getDistanceToVirtualHub() + "," + shooterRPS + "," + hoodAngle);
        //m_shooter.setManualRPS(shooterRPS);
        //m_shooter.setHoodAngle(hoodAngle);
        if (m_shooter.getMode() != Mode.MANUAL) {
            double result = PID.calculate(m_drivetrain.getGyroRadians(), getAngleToTarget());
            m_drivetrain.rotateWithPrivilege(result, 2);
        }
        if (Math.abs(PID.getPositionError()) < Math.toRadians(5)) {
            if (m_shooter.atSetPoint() && m_shooter.getMode() != Mode.OFF) {
                m_hopper.setOn();
                m_feeder.setOn();
            }
            if (isHub) {
                m_shooter.setMode(Mode.HUB);
            } else {
                m_shooter.setMode(Mode.PASS);
            }
        }
        if (Math.abs(PID.getPositionError()) < Math.toRadians(3)) {
            m_drivetrain.setLockedMode(true);
        } else {
            m_drivetrain.setLockedMode(false);
        }
        
        SmartDashboard.putNumber("Angle to Target", getAngleToTarget());
        SmartDashboard.putNumber("Gyro Angle", m_drivetrain.getGyroRadians());
    }

    @Override
    public boolean isFinished() {
        return timeToRun != 0 && System.currentTimeMillis() > endTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.setOff();
        m_shooter.stop();
        m_hopper.setOff();
        m_intake.setAgitateMode(false);
        m_drivetrain.setLockedMode(false);
    }

    public double getAngleToTarget() {
        Pose2d target;
        Pose2d current = RobotContainer.drivetrain.getPose();

        if (m_shooter.getMode() == Mode.HUB) {
            target = RobotContainer.hubPosition;
        } else if (m_shooter.getMode() == Mode.PASS) {
            target = m_drivetrain.getTargetPassPoint();
        } else {
            target = current;
        }
        return (Util.angleBetweenPoses(current, target));
    }
}
