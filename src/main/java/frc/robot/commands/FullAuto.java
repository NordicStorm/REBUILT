package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.Shooter.Mode;

public class FullAuto extends SequentialCommandGroup {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Feeder feeder;
    private Hopper hopper;
    private Intake intake;
    public boolean isInitialized = false;

    public FullAuto(CommandSwerveDrivetrain drivetrain, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        initializeCommands();
    }

    static SendableChooser<String> chooser = new SendableChooser<String>();

    public static void putToDashboard() {
        chooser.addOption("Left", "Left");
        chooser.addOption("Center", "Center");
        chooser.addOption("Right", "Right");
        SmartDashboard.putData(chooser);
    }

    public void initializeCommands() {
        // !WAYFINDER_INFO: {"trackWidth":0.864,"gameName":"Rebuilt"}
        // !WAYFINDER_INFO: {"trackWidth":0.864,"gameName":"Rebuilt"}
        boolean doLastPart = SmartDashboard.getBoolean("DoLastPart?", true);
        boolean isBlue = DriverStation.getAlliance().get() == Alliance.Blue;

        boolean isLeft = chooser.getSelected().equals("Left");
        boolean isCenter = chooser.getSelected().equals("Center");
        boolean isRight = chooser.getSelected().equals("Right");

        DriveTrainConfig config = RobotContainer.drivetrain.getConfig().makeClone();
        config.maxVelocity = 1;
        config.maxAcceleration = 2;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;
        double angleAwayFromWall;
        double angleTowardsFuel;

        if (isBlue) {
            angleAwayFromWall = 0;
            if (isLeft) {
                angleTowardsFuel = -90;
            } else {
                angleTowardsFuel = 90;
            }
        } else {
            angleAwayFromWall = 180;
            if (isLeft) {
                angleTowardsFuel = 90;
            } else {
                angleTowardsFuel = -90;
            }
        }
        if (isCenter) {
            drivetrain.resetRotation(Rotation2d.fromDegrees(angleAwayFromWall));

            MultiPartPath pathB = new MultiPartPath(RobotContainer.drivetrain, config, null);
            pathB.resetPosition(1.623, 3.973);
            pathB.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 2000)); // nomove
            pathB.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove
            pathB.addWaypoint(1.804, 4.761);
            pathB.addParallelCommand(new MoveIntake(intake, false));
            pathB.addWaypoint(1.950, 5.318);
            pathB.addWaypoint(1.804, 6.033);
            pathB.addParallelCommand(new RunIntake(intake, true));
            pathB.addWaypoint(0.593, 5.961);
            pathB.addParallelCommand(new SetShooter(shooter, Mode.HUB));
            pathB.addWaypoint(1.926, 5.852);
            pathB.addWaypoint(1.804, 4.034);
            pathB.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 2000)); // nomove
            pathB.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove
            pathB.addStop();

            if (!isBlue) {
                pathB.flipAllX();
                pathB.flipAllY();
            }

            addCommands(pathB.finalizePath());
        } else {
            drivetrain.resetRotation(Rotation2d.fromDegrees(angleTowardsFuel));

            MultiPartPath pathA = new MultiPartPath(RobotContainer.drivetrain, config, null);
            pathA.resetPosition(4.688, 7.4);
            pathA.setHeading(angleTowardsFuel);
            pathA.addWaypoint(5.658, 7.4);
            pathA.addParallelCommand(new MoveIntake(intake, false));
            pathA.addWaypoint(7.754, 7.221);
            pathA.addParallelCommand(new RunIntake(intake, true));
            pathA.addWaypoint(7.778, 4.967);
            pathA.addWaypoint(7.136, 4.919);
            pathA.addWaypoint(6.457, 5.185);
            pathA.addParallelCommand(new RunIntake(intake, false));
            pathA.setHeading(angleAwayFromWall);
            pathA.addWaypoint(5.852, 7.4);
            pathA.addWaypoint(4.688, 7.4);
            pathA.addParallelCommand(new SetShooter(shooter, Mode.HUB));
            pathA.addWaypoint(3.271, 7.4);
            pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 2000)); // nomove
            pathA.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove
            pathA.setHeading(angleAwayFromWall);
            pathA.addWaypoint(4.725, 7.4);
            pathA.addParallelCommand(new RunIntake(intake, true));
            pathA.addWaypoint(6.324, 7.4);
            pathA.setHeading(angleTowardsFuel);
            pathA.addWaypoint(7.415, 7.4);
            pathA.addWaypoint(7.584, 5.403);
            pathA.addWaypoint(6.457, 5.185);
            pathA.addParallelCommand(new RunIntake(intake, false));
            pathA.setHeading(angleAwayFromWall);
            pathA.addWaypoint(5.852, 7.4);
            pathA.addWaypoint(4.652, 7.4);
            pathA.addParallelCommand(new SetShooter(shooter, Mode.HUB));
            pathA.addWaypoint(3.271, 7.4);
            pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 2000)); // nomove
            pathA.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove
            pathA.addStop();

            if (!isBlue) {
                pathA.flipAllX();
                pathA.flipAllY();
            }

            if (!isLeft) {
                pathA.flipAllY();
            }

            addCommands(pathA.finalizePath());
        }
        isInitialized = true;

    }
}