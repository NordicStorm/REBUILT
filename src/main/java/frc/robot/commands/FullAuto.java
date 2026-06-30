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
import frc.robot.subsystems.PhotonVision;

public class FullAuto extends SequentialCommandGroup {

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Feeder feeder;
    private Hopper hopper;
    private Intake intake;
    public boolean isInitialized = false;
    public PhotonVision camera;

    public FullAuto(CommandSwerveDrivetrain drivetrain, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, PhotonVision camera) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.camera = camera;
        initializeCommands();
    }

    static SendableChooser<String> chooser = new SendableChooser<String>();
    static SendableChooser<Boolean> bumpChooser = new SendableChooser<Boolean>();

    public static void putToDashboard() {
        chooser.addOption("Left", "Left");
        chooser.addOption("Center", "Center");
        chooser.addOption("Right", "Right");
        bumpChooser.addOption("DoBump", true);
        bumpChooser.addOption("No Bump", false);
        SmartDashboard.putData(bumpChooser);
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

        //boolean overBump = chooser.getSelected().equals(true);

        boolean overBump = SmartDashboard.getBoolean("OverBump?", false);

        DriveTrainConfig config = RobotContainer.drivetrain.getConfig().makeClone();
        config.maxVelocity = 2;
        config.maxAcceleration = 3;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;
        double angleAwayFromWall;
        double angleTowardsWall;
        double angleTowardsFuel;
        double diagonalOverBump;

        if (isBlue) {
            angleAwayFromWall = 0;
            angleTowardsWall = 180;
            if (isLeft) {
                diagonalOverBump = -45;
                angleTowardsFuel = -90;
            } else {
                diagonalOverBump = 45;
                angleTowardsFuel = 90;
            }
        } else {
            angleAwayFromWall = 180;
            angleTowardsWall = 0;
            if (isLeft) {
                diagonalOverBump = 135;
                angleTowardsFuel = 90;
            } else {
                diagonalOverBump = 225;
                angleTowardsFuel = -90;
            }
        }
        if (isCenter) {
            drivetrain.resetRotation(Rotation2d.fromDegrees(angleAwayFromWall));

            MultiPartPath pathB = new MultiPartPath(RobotContainer.drivetrain, config, null);
            pathB.resetPosition(3.767, 4.034);
            // pathB.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake,
            // drivetrain, true, 2000)); // nomove
            // pathB.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove
            pathB.changeMaxVelocity(1.25);
            pathB.addWaypoint(1.950, 4.664);
            pathB.addParallelCommand(new MoveIntake(intake, false));
            pathB.addWaypoint(1.950, 5.318);
            pathB.addWaypoint(1.804, 6.033);
            pathB.addParallelCommand(new RunIntake(intake, true, true));
            pathB.addWaypoint(0.593, 5.961);
            pathB.addParallelCommand(new SetShooter(shooter, Mode.HUB));
            pathB.addWaypoint(1.926, 5.852);
            pathB.addWaypoint(2.277, 3.998);
            pathB.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 5000, camera)); // nomove
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
            // pathA.addStop(3000);
            pathA.resetPosition(4.688, 7.000);
            pathA.setHeading(angleTowardsFuel);
            pathA.addWaypoint(6.000, 7.000);
            pathA.addStop(500);
            pathA.addParallelCommand(new MoveIntake(intake, false));
            pathA.addWaypoint(6.500, 7.000);
            pathA.addSequentialCommand(new RunIntake(intake, true, true), 1.25);// nomove
            pathA.addWaypoint(7.524, 7.000);
            pathA.changeMaxVelocity(1.25);
            pathA.setHeadingFollowMovement(0);
            pathA.addWaypoint(7.730, 4.494);
            if (overBump) { // path off
                pathA.addWaypoint(7.039, 4.700);
                pathA.addWaypoint(6.457, 5.185);
                pathA.changeMaxVelocity(2);
                pathA.addSequentialCommand(new RunIntake(intake, false, true), 1.25); // nomove
                pathA.setHeading(diagonalOverBump);
                pathA.addWaypoint(4.761, 5.536);
                pathA.addWaypoint(2.919, 5.536);
                pathA.addWaypoint(2.325, 7);
            } else { // path on
                pathA.addWaypoint(6.954, 4.313);
                pathA.addWaypoint(6.179, 5.052);
                pathA.changeMaxVelocity(2);
                pathA.addSequentialCommand(new RunIntake(intake, false, true), 1.25); // nomove
                pathA.setHeading(angleAwayFromWall);
                pathA.addWaypoint(5.852, 7.000);
                pathA.addWaypoint(4.688, 7.000);
                pathA.addWaypoint(2.325, 7.000);
            }
            pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 5000, camera)); // nomove
            pathA.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove
            pathA.setHeading(angleAwayFromWall);
            pathA.addWaypoint(4.725, 7.000);
            pathA.addWaypoint(6.179, 6.906);
            pathA.changeMaxVelocity(1.25);
            pathA.addSequentialCommand(new RunIntake(intake, true, true), 1.25); // nomove
            pathA.setHeadingFollowMovement(0);
            pathA.addWaypoint(7.875, 6.239);
            pathA.addWaypoint(7.766, 4.216);
            pathA.addWaypoint(6.397, 4.494);
            pathA.addWaypoint(3.658, 4.264);
            // if (overBump) { // path off
            // pathA.addWaypoint(6.457, 5.185);
            // pathA.changeMaxVelocity(2);
            // pathA.addSequentialCommand(new RunIntake(intake, false, true), 1); //
            // nomove
            // pathA.setHeading(diagonalOverBump);
            // pathA.addWaypoint(4.761, 5.536);
            // pathA.addWaypoint(2.919, 5.536);
            // pathA.addWaypoint(2.325, 7);
            // } else { // path on
            // pathA.addWaypoint(6.457, 5.185);
            // pathA.changeMaxVelocity(2);
            // pathA.addSequentialCommand(new RunIntake(intake, false, true), 1); //
            // nomove
            // pathA.setHeading(angleAwayFromWall);
            // pathA.addWaypoint(5.864, 6.966);
            // pathA.addWaypoint(4.652, 7.000);
            // pathA.addWaypoint(2.277, 7.000);
            // }
            // pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake,
            // drivetrain, true, 5000)); // nomove
            // pathA.addSequentialCommand(new SetShooter(shooter, Mode.OFF)); // nomove

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