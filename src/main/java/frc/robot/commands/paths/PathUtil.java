package frc.robot.commands.paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathUtil {
    /**
     * Convert the ChassisSpeeds, which has x and y components, into a single speed number using the pythagorean theorem.
     * @param speeds
     * @return
     */
    public static double linearSpeedFromChassisSpeeds(ChassisSpeeds speeds) {
        return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
    }

     /**
     * angle unit is radians!
     * @param speeds
     * @param radians
     * @return
     */
    public static ChassisSpeeds rotateSpeeds(ChassisSpeeds speeds, double radians){
        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, new Rotation2d(radians));
    }

    
}