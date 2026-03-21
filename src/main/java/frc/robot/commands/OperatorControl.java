package frc.robot.commands;

//import com.ctre.phoenix.Util;
//import frc.robot.Utils.;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.DriveTrainConfig;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OperatorControl extends Command {
    private DriveTrainConfig config;
    private CommandSwerveDrivetrain driveTrain;
    public OperatorControl(CommandSwerveDrivetrain driveTrain) {
        this.config = driveTrain.getConfig();
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        var controller = RobotContainer.m_driverController;

        double forward = -controller.getLeftY();
        double sideways = -controller.getLeftX();
        double rot = -controller.getRightX();

        double throttle = 0.5;//rightStick.getThrottle();
        throttle = Util.map(throttle, 1, -1, 0.1, 1);

        SmartDashboard.putNumber("stick", forward);

        throttle= config.maxVelocity;

        forward = Util.applyDeadzone(forward, 0.1) * throttle;
        sideways = Util.applyDeadzone(sideways, 0.1) * throttle;
        rot = Util.applyDeadzone(rot, 
        0.2);
        rot = Util.signedSquare(rot);
        rot *= 5;

        ChassisSpeeds localSpeeds = Util.rotateSpeeds(new ChassisSpeeds(forward, sideways, rot),
           driveTrain.getGyroRadians() + RobotContainer.AllianceAngleRad);
        if (controller.x().getAsBoolean()) {
            localSpeeds.omegaRadiansPerSecond = Shoot.getRotationVelocity();
        }
        driveTrain.drive(localSpeeds);

    }

}
