package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class PhotonVision extends SubsystemBase {

    private static PhotonCamera fuelCamera;
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
 

    public PhotonVision() {
        fuelCamera = new PhotonCamera("FuelCam");;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    }

    public List<PhotonPipelineResult> getTargets() {
        return fuelCamera.getAllUnreadResults();
    }

    @Override
    public void periodic() {
        var results = fuelCamera.getAllUnreadResults();
        for (var result : results) {
            var multiTagResult = photonEstimator.estimateCoprocMultiTagPose(result);
            if (multiTagResult.isPresent()) {
                RobotContainer.drivetrain.addVisionMeasurement(multiTagResult.get().estimatedPose.toPose2d(), multiTagResult.get().timestampSeconds);
            }
        }
    }
}