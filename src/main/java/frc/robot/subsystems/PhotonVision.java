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
import frc.robot.vision.TargetClusterer;

public class PhotonVision extends SubsystemBase {

    private static PhotonCamera fuelCamera;
    private static final double CAMERA_HEIGHT_INCHES = 0; // TODO
    private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); // TODO

    private List<PhotonPipelineResult> fuel = new ArrayList<PhotonPipelineResult>();

    public PhotonVision() {
        fuelCamera = new PhotonCamera("FuelCam");
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

                double eps = 30; // degrees
                int minPts = 5; // minimum points to form a cluster
                double areaTol = 1.3; // 30% allowed difference

                List<List<PhotonTrackedTarget>> clusters = TargetClusterer.clusterByAngularAndArea(pts, eps, minPts, areaTol);
                SmartDashboard.putString("PhotonVision", String.format("Found %d clusters", clusters.size()));
                for (int i = 0; i < clusters.size(); i++) {
                    //SmartDashboard.putString("PhotonVisionCluster: " + i, clusters.get(i).toString());
                }
            }
        }
    }
}