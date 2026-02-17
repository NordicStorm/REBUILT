package frc.robot.commands.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class DummyPath {
    /**
     * This loads trajectories and fake-inits them
     */
    static void fakeSetup() {
        var drivetrain = new FakeDrivetrain();
        MultiPartPath pathA = new MultiPartPath(drivetrain);
        pathA.addWaypoint(5, 5);
        pathA.addWaypoint(10, 10);
        pathA.addStop();
        List<WaypointPiece> wps = new ArrayList<>();
        wps.add(new WaypointPiece(5, 5));
        wps.add(new WaypointPiece(10, 10));
        var path = pathA.finalizePath();
        var traj = new TrajectoryFollowPiece(drivetrain, wps, 0, pathA);
        traj.initialize();
    }
    private static class FakeDrivetrain extends SubsystemBase implements PathableDrivetrain {

        @Override
        public double getGyroRadians() {
            return 0;
        }

        @Override
        public Pose2d getPose() {
            return new Pose2d();
        }

        @Override
        public void setPose(Pose2d pose) {
            
        }

        @Override
        public void drive(ChassisSpeeds speeds) {
            
        }

        @Override
        public ChassisSpeeds getSpeeds() {
            return new ChassisSpeeds(1, 1, 0);
        }

        @Override
        public DriveTrainConfig getConfig() {
            DriveTrainConfig config = new DriveTrainConfig();
            config.maxVelocity = 4.5;
            config.maxAcceleration = 3;
            config.maxAnglularVelocity = 10;
            config.maxAngularAcceleration = 8;
            config.maxCentripetalAcceleration = 7;

            config.rotationCorrectionP = 10;
            return config;
        }

    }
    
}