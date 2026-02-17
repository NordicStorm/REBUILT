package frc.robot.commands.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public class TrajectoryFollowPiece extends Command implements CommandPathPiece {

    private List<Pair<Double, Command>> commandTriggerTimes = new ArrayList<>();
    private boolean done = false;
    private PathableDrivetrain drivetrain;
    private List<WaypointPiece> waypoints;
    private MultiPartPath path;
    private double endVelocity;
    private Trajectory trajectory;
    private DriveTrainConfig drivetrainConfig;
    private long startTime;
    private HolonomicDriveController controller;

    /**
     * 
     * @param drivetrain  the drivetrain to use
     * @param waypoints   a list of waypoints to go to, in order, after the current
     *                    position.
     * @param endVelocity the speed the robot should be going right at the end of
     *                    the trajectory. Meters per second.
     */
    TrajectoryFollowPiece(PathableDrivetrain drivetrain, List<WaypointPiece> waypoints, double endVelocity,
            MultiPartPath path) {
        this.drivetrain = drivetrain;
        this.waypoints = waypoints;
        this.endVelocity = endVelocity;
        this.path = path;
        this.drivetrainConfig = path.getDrivetrainConfig();
        addRequirements((Subsystem) drivetrain);
    }

    @Override
    public double getRequestedStartSpeed() {
        return 0; // TODO?
    }

    @Override
    public void initialize() {
        done = false;
        Pose2d currentPose = drivetrain.getPose();
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();
        double speed = PathUtil.linearSpeedFromChassisSpeeds(currentSpeeds);
        ChassisSpeeds globalSpeeds = PathUtil.rotateSpeeds(currentSpeeds, -drivetrain.getGyroRadians());
        List<Translation2d> interiorPoints = new ArrayList<>();
        Rotation2d startMovementDirection = null;

        var end = waypoints.get(waypoints.size() - 1);
        var endPosition = end.getPoint();
        if (Math.abs(speed) < 1) {
            Translation2d firstPoint; // calculate the direction from the second-to-last to the last point.

            if (interiorPoints.size() > 0) {
                firstPoint = interiorPoints.get(0);
            } else {
                firstPoint = endPosition;
            }
            startMovementDirection = new Rotation2d(firstPoint.getX() - currentPose.getX(),
                    firstPoint.getY() - currentPose.getY());

        } else {
            // the time to stop times 1/2 to allow curve
            /*double futureMultiplier = 2.5 / drivetrainConfig.maxAcceleration;
            Pose2d futurePose = currentPose
                    .plus(new Transform2d(new Translation2d(globalSpeeds.vxMetersPerSecond * futureMultiplier,
                            globalSpeeds.vyMetersPerSecond * futureMultiplier), new Rotation2d()));*/
            // interiorPoints.add(futurePose.getTranslation());
            startMovementDirection = new Rotation2d(globalSpeeds.vxMetersPerSecond, globalSpeeds.vyMetersPerSecond);
        }
        currentPose = new Pose2d(currentPose.getTranslation(), startMovementDirection);

        for (int i = 0; i < waypoints.size() - 1; i++) { // don't add the last waypoint, it is not interior.
            interiorPoints.add(waypoints.get(i).getPoint());
        }

        double endDirection;
        if (end.forcedEndDirection == null) {
            Translation2d from; // calculate the direction from the second-to-last to the last point.
            if (interiorPoints.size() > 0) {
                from = interiorPoints.get(interiorPoints.size() - 1);
            } else {
                from = currentPose.getTranslation();
            }
            endDirection = Math.atan2(endPosition.getY() - from.getY(), endPosition.getX() - from.getX());
        } else {
            endDirection = end.forcedEndDirection;
        }
        Pose2d endPose = new Pose2d(endPosition, new Rotation2d(endDirection));
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(drivetrainConfig.maxVelocity,
                drivetrainConfig.maxAcceleration);
        trajectoryConfig.setStartVelocity(speed);
        trajectoryConfig
                .addConstraint(new CentripetalAccelerationConstraint(drivetrainConfig.maxCentripetalAcceleration));
        trajectoryConfig.setEndVelocity(endVelocity);
        trajectory = TrajectoryGenerator.generateTrajectory(currentPose, interiorPoints, endPose, trajectoryConfig);

        List<State> states = trajectory.getStates();
        int j = 0;
        for (int i = 0; i < waypoints.size(); i++) {
            while (!states.get(j).poseMeters.getTranslation().equals(waypoints.get(i).getPoint())){ // loop until the state = the waypoint
                j++;
            }
            if (waypoints.get(i).parallelCommands.size() > 0) {
                SequentialCommandGroup group = new SequentialCommandGroup();
                for (CommandPathPiece command : waypoints.get(i).parallelCommands) {
                    group.addCommands((Command) command);
                }
                commandTriggerTimes
                        .add(new Pair<>(Double.valueOf(states.get(j).timeSeconds), group));
            }

        }

        controller = new HolonomicDriveController(
                new PIDController(drivetrainConfig.positionCorrectionP, drivetrainConfig.positionCorrectionI,
                        drivetrainConfig.positionCorrectionD),
                new PIDController(drivetrainConfig.positionCorrectionP, drivetrainConfig.positionCorrectionI,
                        drivetrainConfig.positionCorrectionD),
                path.getRotationController()); // uses the path's rotation controller for consistency
        controller.setTolerance(new Pose2d(
                new Translation2d(drivetrainConfig.endOfTrajectoryPositionTolerance,
                        drivetrainConfig.endOfTrajectoryPositionTolerance),
                new Rotation2d(drivetrainConfig.endOfTrajectoryAngleTolerance)));
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double timeProgressSeconds = (System.currentTimeMillis() - startTime) / 1000.0; // ms to seconds
        State goal = trajectory.sample(timeProgressSeconds);
        Pose2d currentPose = drivetrain.getPose();

        for (int i = commandTriggerTimes.size() - 1; i >= 0; --i) { // iterate backwards so we can remove stuff while
                                                                    // iterating
            var trigger = commandTriggerTimes.get(i);
            if (timeProgressSeconds >= trigger.getFirst()) { // if we have passed that time, it is at that waypoint and
                                                             // should start.
                trigger.getSecond().schedule();
                commandTriggerTimes.remove(i);
            }
        }

        if (path.headingFollowMovement) {
            path.targetRotationDegrees = path.headingOffset + goal.poseMeters.getRotation().getDegrees();
        }

        double targetRotation = Math.toRadians(path.targetRotationDegrees);

        ChassisSpeeds speeds = controller.calculate(currentPose, goal, new Rotation2d(targetRotation));
        drivetrain.drive(speeds);
        //System.out.println(timeProgressSeconds);
        if (timeProgressSeconds >= trajectory.getTotalTimeSeconds()) {
            if (controller.atReference()) {
                done = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return done;
    }

}