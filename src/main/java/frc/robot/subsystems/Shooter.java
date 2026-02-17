package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private final TalonFX m_shooter = new TalonFX(MechanismConstants.kShooter1ID, "CANivore");

    //
    // State
    //
    private ControlMode m_ControlMode = ControlMode.kStop;
    private double m_speed = 0;

    //
    // PID
    //

    public Shooter() {

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

        m_shooter.getConfigurator().apply(shooterSlot0Configs);

        m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimit = 80;

        m_shooter.getConfigurator().apply(m_shooterConfig);
      
        m_shooter.optimizeBusUtilization();
    }

    public void setSpeed(double speed) {
        m_speed = speed;
    }

    private void setShooterRPM(double rpm) {
        m_shooter.setControl(new VelocityVoltage(rpm).withSlot(0));
    }

    public Command shoot(double rpm) {
        return Commands.startEnd(() -> this.setShooterRPM(rpm), () -> this.setShooterRPM(0), this);
    }

    public void stop() {
        setShooterRPM(0);
    }

    @Override
    public void periodic() {
        //m_shooter.set(m_speed);
        SmartDashboard.putNumber("Velocity from Encoder", m_shooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooting Value", RobotContainer.shootingSpeed);
    }
}
