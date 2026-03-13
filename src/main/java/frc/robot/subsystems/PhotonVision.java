package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Cluster;
import frc.robot.vision.TargetClusterer;
import frc.robot.vision.TemporalClusterSmoother;

public class PhotonVision extends SubsystemBase {

    private static PhotonCamera fuelCamera;
    private static final double CAMERA_HEIGHT_INCHES = 0; // TODO
    private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); // TODO

    private List<PhotonPipelineResult> fuel = new ArrayList<PhotonPipelineResult>();
    private final TemporalClusterSmoother clusterSmoother;

    public PhotonVision() {
        fuelCamera = new PhotonCamera("FuelCam");
        // Initialize temporal smoother: alpha=0.3, matchThresholdDeg=2.0, areaTol=1.25, maxMissedFrames=3
        clusterSmoother = new TemporalClusterSmoother(0.3, 2.0, 1.25, 3);
    }

    public List<PhotonPipelineResult> getTargets() {
        return fuelCamera.getAllUnreadResults();
    }

    @Override
    public void periodic() {
        fuel = getTargets();
        if (!fuel.isEmpty()) {
            var result = fuel.get(fuel.size() - 1);
            if (result.hasTargets()) {
                List<PhotonTrackedTarget> pts = result.getTargets();

                double eps = 7.5; // degrees
                int minPts = 3; // minimum points to form a cluster
                double areaTol = 1.75; // 50% allowed difference
               

                List<Cluster> clusters = TargetClusterer.clusterByAngularAndArea(pts, eps, minPts, areaTol);
                // run temporal smoothing (keeps state across frames inside clusterSmoother)
                var smoothed = clusterSmoother.update(clusters);
                SmartDashboard.putString("PhotonVision", String.format("Found %d clusters", clusters.size()));
                SmartDashboard.putString("PhotonVisionSmoothed", String.format("Found %d smoothed", smoothed.size()));
                for (int i = 0; i < smoothed.size(); i++) {
                    TemporalClusterSmoother.SmoothedCluster cluster = smoothed.get(i);
                    SmartDashboard.putString("PhotonVisionSmoothedCluster: " + i, "Size: " + cluster.size + ", Area: " + String.format("%.2f", cluster.area) + ", Pitch: " + String.format("%.2f", cluster.pitch) + ", Yaw: " + String.format("%.2f", cluster.yaw));
                }
            }
        }
    }
}