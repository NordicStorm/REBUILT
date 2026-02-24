// Intake deploy x44
// Intake motor x44
// Roller floor x60

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PhotonVision;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public final Orchestra music = new Orchestra();
    private final PhotonVision fuelCamera = new PhotonVision();
    public static double shootingSpeed = .5;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_secondController = new CommandXboxController(1);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .15; // TODO we changed value
                                                                                        // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        music.loadMusic("output.chrp");
        music.addInstrument(m_feeder.m_feeder);
        music.addInstrument(m_shooter.m_shooter);
        for (int i = 0; i < 4; i++) {
            music.addInstrument(drivetrain.getModule(i).getDriveMotor());
            music.addInstrument(drivetrain.getModule(i).getSteerMotor());
        }
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(MaxSpeed * -m_driverController.getLeftY())
                        .withVelocityY(MaxSpeed * -m_driverController.getLeftX())
                        .withRotationalRate(-m_driverController.getRightX())));

        m_driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.setControl(brake)));
        m_driverController.rightBumper().and(m_driverController.leftBumper())
                .onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));

        m_driverController.leftTrigger().whileTrue(new Shoot(m_shooter, m_feeder));
        m_driverController.b().onTrue(new InstantCommand(() -> music.play(), m_feeder, m_shooter));
        m_driverController.b().onFalse(new InstantCommand(() -> music.stop(), m_feeder, m_shooter));

        m_driverController.povDown().onTrue(new InstantCommand(() -> shootingSpeed = shootingSpeed - 10));
        m_driverController.povUp().onTrue(new InstantCommand(() -> shootingSpeed = shootingSpeed + 10));

    }
}