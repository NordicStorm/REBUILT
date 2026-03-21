package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Util {
    /**
     * angle unit is radians!
     * 
     * @param speeds
     * @param radians
     * @return
     */
    public static ChassisSpeeds rotateSpeeds(ChassisSpeeds speeds, double radians) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, new Rotation2d(radians));
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        } else {
            return Math.copySign(map(Math.abs(value), deadzone, 1, 0, 1), value);
        }
    }

    public static double absClamp(double val, double max) {
        if (val < -max) {
            return -max;
        }
        if (val > max) {
            return max;
        }
        return val;
    }

    public static double lerp(double a, double b, double f) {
        return (a * (1.0 - f)) + (b * f);
    }

    public static double clamp(double n, double min, double max) {
        return Math.max(Math.min(n, max), min);
    }

    public static double minWithAbs(double a, double b) {
        return Math.abs(a) < Math.abs(b) ? a : b;
    }

    public static double maxWithAbs(double a, double b) {
        return Math.abs(a) > Math.abs(b) ? a : b;
    }

    public static double signedSquare(double x) {
        if (x < 0)
            return -x * x;
        else
            return x * x;
    }

    /**
     * Positive is CCW. Return radians
     * 
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     */
    public static double angleBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    public static double angleBetweenPoses(Pose2d start, Pose2d end) {
        return angleBetweenPoints(start.getX(), start.getY(), end.getX(), end.getY());
    }

    /**
     * 
     * @param current degrees!
     * @param target
     * @return degrees of difference. Positive means that the target is greater than
     *         the current
     */
    public static double angleDiff(double current, double target) {
        current = current % 360 - 180;
        target = target % 360 - 180;
        double diff = (target - current);
        if (diff > 180) {
            diff = -360 + diff;
        }
        if (diff < -180) {
            diff = 360 + diff;
        }
        return diff;
    }

    /**
     * Returns true if a and b are within tolerance of each other
     * 
     * @param a
     * @param b
     * @param tolerance will pass if diff is <= tolerance
     * @return
     */
    public static boolean close(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    /**
     * Get distance in meters between 2 poses
     * 
     * @return
     */
    public static double distance(Pose2d a, Pose2d b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
