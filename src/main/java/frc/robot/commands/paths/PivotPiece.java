package frc.robot.commands.paths;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotPiece extends Command implements CommandPathPiece {

    private PathableDrivetrain drivetrain;
    private DriveTrainConfig drivetrainConfig;
    private ProfiledPIDController controller;
    private double targetAngle;

    PivotPiece(PathableDrivetrain drivetrain, double targetAngle, MultiPartPath path) {
        this.drivetrain = drivetrain;
        this.targetAngle = Math.toRadians(targetAngle);
        this.drivetrainConfig = path.getDrivetrainConfig();
        controller = path.getRotationController();
    }

    @Override
    public double getRequestedStartSpeed() {
        return 0;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(0, 0, controller.calculate(drivetrain.getGyroRadians(), targetAngle)));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));

    }

    @Override
    public boolean isFinished() {
        return controller.getPositionError() < Math.toRadians(5)
                && controller.getVelocityError() < drivetrainConfig.stopAngularVelocityTolerance;
    }

}