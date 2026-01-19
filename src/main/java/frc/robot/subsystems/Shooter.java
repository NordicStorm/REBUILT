package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

    private final SparkMax mVortexMotor = new SparkMax(20, MotorType.kBrushless);
    private SparkMaxConfig m_Config = new SparkMaxConfig();
    private SparkClosedLoopController m_pidController;

    private double m_speed = 0;

    public Shooter() {
        m_Config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50);
        m_Config.closedLoop
                .p(Constants.ShooterConstants.kP)
                .i(Constants.ShooterConstants.kI)
                .d(Constants.ShooterConstants.kD);

        mVortexMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_pidController = mVortexMotor.getClosedLoopController();
    }

    public void setSpeed(double speed) {
        m_speed = speed;
    }

    private void setRPM(double rpm) {
        m_pidController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public Command shoot(double rpm) {
        return Commands.startEnd(() -> this.setRPM(rpm), () -> this.setSpeed(0), this);
    }

    private Command openLoopShootingCommand(DoubleSupplier IntakeVoltageSupplier) {
        return Commands.startEnd(
                () -> this.setSpeed(IntakeVoltageSupplier.getAsDouble()), () -> this.setSpeed(0), this);
    }

    public Command openLoopShootingCommand(double IntakeVoltage) {
        return openLoopShootingCommand(() -> IntakeVoltage);
    }

    @Override
    public void periodic() {
        mVortexMotor.set(m_speed);
        SmartDashboard.putNumber("Velocity from Encoder", mVortexMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Voltage %", m_speed);
        SmartDashboard.putNumber("Shooting Value", RobotContainer.shootingSpeed);
    }
}
