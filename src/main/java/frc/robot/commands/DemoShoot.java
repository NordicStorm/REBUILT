package frc.robot.commands;

import java.util.List;

import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Mode;

public class DemoShoot extends Command {
    private Shooter m_shooter;
    private Feeder m_feeder;
    private Hopper m_hopper;
    private Intake m_intake;
    private CommandSwerveDrivetrain m_drivetrain;
    private ProfiledPIDController PID;
    private PhotonVision camera;
    private boolean seeTarget = false;
    private PhotonTrackedTarget targetTag;

    public DemoShoot(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, CommandSwerveDrivetrain drivetrain, PhotonVision camera) {
        this.m_feeder = feeder;
        this.m_shooter = shooter;
        this.m_hopper = hopper;
        this.m_intake = intake;
        this.m_drivetrain = drivetrain;
        this.camera = camera;
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

      //  m_intake.setAgitateMode(true);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Demo PID Position Error", PID.getPositionError());
        m_shooter.setMode(Mode.DEMO);

        // m_shooter.setManualRPS(shooterRPS);
        // m_shooter.setHoodAngle(hoodAngle);
        PhotonTrackedTarget target = getTarget();
        double angleToTarget = Math.toRadians(getAngleToTarget());
        double distanceToTarget = getDistanceToTarget();
        if (m_shooter.getMode() != Mode.MANUAL && seeTarget) {
            double result = PID.calculate(m_drivetrain.getGyroRadians(), getAngleToTarget());
            m_drivetrain.rotateWithPrivilege(result, 2);
        }
        /*if (Math.abs(angleToTarget) <= Math.toRadians(5)) {
            if (m_shooter.atSetPoint() && m_shooter.getMode() != Mode.OFF) {
                m_hopper.setOn();
                m_feeder.setOn();
            }
        }
        if (Math.abs(angleToTarget) <= Math.toRadians(3)) {
            m_drivetrain.setLockedMode(true);
        } else {
            m_drivetrain.setLockedMode(false);
        }*/

        SmartDashboard.putNumber("Demo Angle to Target", getAngleToTarget());
        SmartDashboard.putBoolean("Demo see target", seeTarget);
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.setOff();
        m_shooter.stop();
        m_hopper.setOff();
        m_intake.setAgitateMode(false);
        m_drivetrain.setLockedMode(false);
    }

    public PhotonTrackedTarget getTarget() {
        List<PhotonTrackedTarget> targets = camera.getTags();

        for (PhotonTrackedTarget target : targets) {
            int targetID = target.getFiducialId();
            if (targetID == 26) {
                seeTarget = true;
                targetTag = target;
                break;
            }
        }
        return (targetTag);
    }

    public double getAngleToTarget() {
        getTarget();
        boolean seeTarget = false;
        double angle = 361;
        
        if (seeTarget) {
           angle = targetTag.getYaw();
        }

        return (Math.toRadians(angle));
    }

    public double getDistanceToTarget() {
        getTarget();
        PhotonTrackedTarget target = this.targetTag;
        if (seeTarget) {
            double yaw = target.getYaw();
            double scale = target.area;
            SmartDashboard.putNumber("Demo Yaw", yaw);
            SmartDashboard.putNumber("Demo Area", scale);       
        }

        double distance = 0;

        return (distance);

    }
}
