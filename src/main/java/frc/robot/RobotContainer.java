// Intake deploy x44
// Intake motor x44
// Roller floor x60

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.FullAuto;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.OperatorControl;
import frc.robot.commands.RunIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Intake m_intake = new Intake();
    private final Hopper m_hopper = new Hopper();
    private FullAuto m_autos;

    // private final PhotonVision fuelCamera = new PhotonVision();
    public static double shootingSpeed = .5;
    public static Pose2d hubPosition, topPassingTarget, bottomPassingTarget;;;;;;;;;;;
    public static double AllianceAngleRad;
    public static boolean isBlue;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_secondController = new CommandXboxController(1);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .45; // TODO we changed value
                                                                                        // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        FullAuto.putToDashboard();
        configureBindings();
        SmartDashboard.putBoolean("Is auto initialized?", false);
        SmartDashboard.putData("Set Auto", new InstantCommand(() -> {
            m_autos = new FullAuto(drivetrain, m_shooter, m_feeder, m_hopper, m_intake);
            SmartDashboard.putBoolean("Is auto initialized?", m_autos.isInitialized);
        }).ignoringDisable(true));
        isBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        if (isBlue) {
            hubPosition = new Pose2d(4.628, 8.069263 / 2, Rotation2d.kZero);
            topPassingTarget = new Pose2d(2.871, 6.01, Rotation2d.kZero);
            bottomPassingTarget = new Pose2d(2.871, 2.059, Rotation2d.kZero);
            AllianceAngleRad = 0;
        } else {
            hubPosition = new Pose2d(11.913, 8.069263 / 2, Rotation2d.fromDegrees(180));
            topPassingTarget = new Pose2d(13.67, 6.01, Rotation2d.kZero);
            bottomPassingTarget = new Pose2d(13.67, 2.059, Rotation2d.kZero);
            AllianceAngleRad = Rotation2d.k180deg.getRadians();
        }
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new OperatorControl(drivetrain));

        // m_driverController.a().onTrue(drivetrain.runOnce(() ->
        // drivetrain.setControl(brake)));
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));

        m_driverController.leftBumper().onTrue(new MoveIntake(m_intake, false).andThen(new RunIntake(m_intake, true)));
        m_driverController.leftBumper().onFalse(new RunIntake(m_intake, false));
        m_driverController.rightBumper().onTrue(new
         RunIntake(m_intake, false).andThen(new MoveIntake(m_intake, true)));
        m_driverController.a().onTrue(new InstantCommand(() -> m_shooter.setHoodAngle((int) SmartDashboard.getNumber("Hood Pulse Request", 1100))));

        m_driverController.rightTrigger().whileTrue(new AutoShoot(m_shooter, m_feeder, m_hopper, drivetrain, true, 0));
    }
}