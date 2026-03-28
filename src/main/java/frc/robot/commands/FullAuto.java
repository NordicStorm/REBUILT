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
        drivetrain.resetRotation(drivetrain.getOperatorForwardDirection().plus(Rotation2d.fromDegrees(0)));

        boolean isLeft = chooser.getSelected().equals("Left");
        boolean isCenter = chooser.getSelected().equals("Center");
        boolean isRight = chooser.getSelected().equals("Right");

        RobotContainer.drivetrain.resetAngle();

        DriveTrainConfig config = RobotContainer.drivetrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 4;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;

        MultiPartPath pathA = new MultiPartPath(RobotContainer.drivetrain, config, null);
        pathA.addWaypoint(4.688, 7.499);
        pathA.addWaypoint(5.658, 7.451);
        pathA.addParallelCommand(new MoveIntake(intake, false));
        pathA.setHeading(-90);
        pathA.addWaypoint(7.754, 7.221);
        pathA.addParallelCommand(new RunIntake(intake, true));
        pathA.addWaypoint(7.778, 4.967);
        pathA.addWaypoint(7.136, 4.919);
        pathA.addWaypoint(6.457, 5.185);
        pathA.addParallelCommand(new RunIntake(intake, false));
        pathA.addWaypoint(5.852, 7.512);
        pathA.addWaypoint(4.531, 7.451);
        pathA.addParallelCommand(new SetShooter(shooter, Mode.HUB));
        pathA.addWaypoint(3.271, 7.560);
        pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 2000)); // nomove
        pathA.addParallelCommand(new SetShooter(shooter, Mode.OFF));
        pathA.addWaypoint(4.725, 7.524);
        pathA.addParallelCommand(new RunIntake(intake, true));
        pathA.addWaypoint(7.415, 7.378);
        pathA.addWaypoint(7.584, 5.403);
        pathA.addWaypoint(6.457, 5.185);
        pathA.addParallelCommand(new RunIntake(intake, false));
        pathA.addWaypoint(5.852, 7.512);
        pathA.addWaypoint(4.531, 7.451);
        pathA.addParallelCommand(new SetShooter(shooter, Mode.HUB));
        pathA.addWaypoint(3.271, 7.560);
        pathA.addSequentialCommand(new AutoShoot(shooter, feeder, hopper, intake, drivetrain, true, 2000)); // nomove
        pathA.addParallelCommand(new SetShooter(shooter, Mode.OFF));
        pathA.addStop();

        if (!isBlue) {
            pathA.flipAllX();
            pathA.flipAllY();
        }

        if (!isLeft) {
            pathA.flipAllY();
        }
        
        addCommands(pathA.finalizePath());
        isInitialized = true;
    }
}