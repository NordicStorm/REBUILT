package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FullAuto extends SequentialCommandGroup {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Feeder feeder;
    private Hopper hopper;
    private Intake intake;

    public FullAuto(CommandSwerveDrivetrain drivetrain, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        initializeCommands();
    }

    public void initializeCommands() {
        // !WAYFINDER_INFO: {"trackWidth":0.864,"gameName":"Rebuilt"}
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
        pathA.resetPosition(3.635, 7.560);
        pathA.addParallelCommand(new MoveIntake(intake, false));
        pathA.addSequentialCommand(new SetShooter(shooter, Shooter.Mode.HUB)); // nomove
        pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, drivetrain, true)); // nomove
        pathA.addSequentialCommand(new SetShooter(shooter, Shooter.Mode.OFF)); // nomove
        pathA.addWaypoint(5.982, 7.365);
        pathA.addWaypoint(7.539, 7.388);
        pathA.addParallelCommand(new RunIntake(intake, true));
        pathA.addWaypoint(7.734, 4.560);
        pathA.addWaypoint(6.222, 7.365);
        pathA.addParallelCommand(new RunIntake(intake, false));
        pathA.addWaypoint(5.284, 7.514);
        pathA.addWaypoint(4.929, 7.502);
        pathA.addWaypoint(2.719, 7.262);
        pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, drivetrain, true)); // nomove
        pathA.addWaypoint(3.441, 7.365);
        pathA.addWaypoint(5.639, 7.422);
        pathA.addWaypoint(7.562, 7.125);
        pathA.addParallelCommand(new RunIntake(intake, true));
        pathA.addWaypoint(7.779, 4.434);
        pathA.addWaypoint(6.692, 7.010);
        pathA.addParallelCommand(new RunIntake(intake, false));
        pathA.addWaypoint(5.169, 7.445);
        pathA.addWaypoint(3.086, 7.296);
        pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, drivetrain, true)); // nomove
        pathA.addStop();
        addCommands(pathA.finalizePath());
    }
}