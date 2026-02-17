package frc.robot.commands.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface PathableDrivetrain {
    /**
     * Get the current angle of the robot, probably from a gyro.
     * 
     * @return the angle in <b>radians</b>. Positive = counterclockwise!
     */
    double getGyroRadians();

    /**
     * Get the current pose of the robot, which includes position and rotation.
     * Rotation is in radians.
     * 
     * @return a Pose2d, probably from an Odometry object.
     */
    Pose2d getPose();

    /**
     * Reset the pose to the given one. Only should set the translation, not whole rotation!!
     * @param pose
     */
    void setPose(Pose2d pose);

    /**
     * Set the speeds of the robot. Uses a ChassisSpeeds object, with speeds given
     * in meters per second and radians per second.
     * 
     * @param speeds A ChassisSpeeds object, should be converted to modules speeds
     *               via a SwerveDriveKinematics object
     */
    void drive(ChassisSpeeds speeds);

    /**
     * Get the actual, current speeds of the robot. This includes x, y, and
     * rotation.
     * 
     * @return the current speeds of the robot. This should probably come from a
     *         SwerveDriveKinematics object
     */
    ChassisSpeeds getSpeeds();

    /**
     * Get the DrivetrainConfig of this drivetrain.
     * @return this drivetrain's config
     */
    DriveTrainConfig getConfig();
}