package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightVision extends SubsystemBase {

    @Override
    public void periodic() {
        // First, tell Limelight your robot's current orientation
        double robotYaw = RobotContainer.drivetrain.getGyroDegrees();
        LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        if(limelightMeasurement == null) return;
        // Add it to your pose estimator
        if (limelightMeasurement.tagCount >= 1) {
            RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.9, .9, 9999999));
            RobotContainer.drivetrain.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
        }
        SmartDashboard.putNumber("Limelight Time", LimelightHelpers.getLimelightNTDouble("", "hb"));

    }
    public boolean isValid() {
        return Timer.getFPGATimestamp() - LimelightHelpers.getLatestResults("").timestamp_RIOFPGA_capture < 2;
    }

    public int seenTagID() {
        return Math.max(0, (int) LimelightHelpers.getFiducialID(""));
    }

    public double getXOffset(){
        return LimelightHelpers.getTX("");
    }


}