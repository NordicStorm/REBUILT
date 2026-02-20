// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class MechanismConstants {
        public static final int kShooter1ID = 11;
        public static final int kFeederMotorID = 10;
        public static final int kRetractLimitSwitchChannel = 9;
        public static final double kRotationToInches = .418; // 1.0/20.0 * ((68.8/25.4)/2) * Math.PI * 2.0;//TODO: Find
                                                             // the correct value
    }

    public static class ShooterConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }
}