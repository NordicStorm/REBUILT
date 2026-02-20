package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

    private enum ControlMode {
        kStop, kPID
    }

    //
    // Hardware
    //
    private final TalonFXConfiguration m_shooterConfig = new TalonFXConfiguration();
    private final TalonFX m_shooter = new TalonFX(11, "rio");
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);


    //
    // State
    //
    private ControlMode m_ControlMode = ControlMode.kStop;
    private double m_speed = 0;

    //
    // PID
    //

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPS Request", 0);

        /*
         * Kg - output to overcome gravity (output)
         * Ks - output to overcome static friction (output)
         * Kv - output per unit of requested velocity (output/rps)
         * Ka - unused, as there is no target acceleration
         * Kp - output per unit of error in velocity (output/rps)
         * Ki- output per unit of integrated error in velocity (output/rotation)
         * Kd - output per unit of error derivative in velocity (output/(rps/s))
         */

        var shooterSlot0Configs = new Slot0Configs();
        shooterSlot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        shooterSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterSlot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        shooterSlot0Configs.kI = 0; // no output for integrated error
        shooterSlot0Configs.kD = 0; // no output for error derivative

        m_shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimit = 80;

        m_shooter.getConfigurator().apply(m_shooterConfig);
        m_shooter.getConfigurator().apply(shooterSlot0Configs);
      
        m_shooter.optimizeBusUtilization();
    }

    public void setRPM(double RPM) {
        m_speed = RPM/60;
    }

    private void setShooterRPM(double rpm) {
        m_shooter.setControl(velocityRequest.withVelocity(rpm));
    }

    public Command shoot(double rpm) {
        return Commands.startEnd(() -> this.setShooterRPM(rpm), () -> this.setShooterRPM(0), this);
    }

    public void stop() {
        setRPM(0);
    }

    public void setVoltage(double v) {
        m_shooter.setVoltage(v);
    }

    public void periodic() {
        setShooterRPM(m_speed);
        SmartDashboard.putNumber("Shooter Velocity", m_shooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooting Value", m_speed);
        SmartDashboard.putNumber("Requested PID", m_shooter.getMotorVoltage().getValueAsDouble());
    }
}
