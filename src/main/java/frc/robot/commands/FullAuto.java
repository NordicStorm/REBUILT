package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.MultiPartPath;

public class FullAuto extends SequentialCommandGroup {

    public FullAuto() {
        initializeCommands();
    }

    public void initializeCommands() {
        // !WAYFINDER_INFO: {"trackWidth":0.864,"gameName":"Rebuilt"}
        boolean doLastPart = SmartDashboard.getBoolean("DoLastPart?", true);
        RobotContainer.drivetrain.resetAngle();

        DriveTrainConfig config = RobotContainer.drivetrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 4;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;

        MultiPartPath pathA = new MultiPartPath(RobotContainer.drivetrain, config, null);
        pathA.resetPosition(0.381, 7.000);
        pathA.addWaypoint(2.050, 7.000);
        pathA.addWaypoint(4.666, 8.063);
        if (doLastPart) {// path on
            pathA.addWaypoint(4.430, 5.364);
            pathA.addWaypoint(4.803, 4.011);
        }
        pathA.addStop();
        addCommands(pathA.finalizePath());
    }
}