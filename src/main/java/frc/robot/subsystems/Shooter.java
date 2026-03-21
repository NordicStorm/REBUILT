package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;

public class Shooter extends SubsystemBase {

    //
    // Hardware
    //
    private final TalonFXConfiguration m_shooterConfig = new TalonFXConfiguration();
    public final TalonFX m_shooter = new TalonFX(Constants.MechanismConstants.kShooter1ID, "rio");
    private final ServoHub m_ServoHub = new ServoHub(Constants.MechanismConstants.kServoHubID);
    private final ServoHubConfig m_servoHubConfig = new ServoHubConfig();
    private final ServoChannel m_hoodServo = m_ServoHub.getServoChannel(ChannelId.kChannelId1);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private double m_speed = 0;
    private int m_hoodServoPulseWidth = 1000; // TODO

    //
    // PID
    //

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPS Request", 0);
        SmartDashboard.putNumber("Hood Pulse Request", 1000);

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

        var shooterSlot1Configs = new Slot1Configs();
        shooterSlot1Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        shooterSlot1Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterSlot1Configs.kP = 0.05; // An error of 1 rps results in 0.11 V output
        shooterSlot1Configs.kI = 0; // no output for integrated error
        shooterSlot1Configs.kD = 0; // no output for error derivative

        m_shooterConfig.Slot0 = shooterSlot0Configs;
        m_shooterConfig.Slot1 = shooterSlot1Configs;

        m_shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimit = 80;

        m_shooter.getConfigurator().apply(m_shooterConfig);
        m_shooter.getConfigurator().apply(shooterSlot0Configs);

        m_shooter.optimizeBusUtilization();

        m_servoHubConfig.channel0
                .pulseRange(1000, 1500, 2000)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        m_ServoHub.configure(m_servoHubConfig, ServoHub.ResetMode.kResetSafeParameters);
        m_hoodServo.setPowered(true);
        m_hoodServo.setEnabled(true);

        m_ServoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 10000);
    }

    public void setRPM(double RPM) {
        m_speed = RPM / 60;
    }

    private void setShooterRPM(double rpm) {
        if (rpm == 0) {
            m_shooter.setControl(velocityRequest.withVelocity(0).withSlot(1));
        } else {
            m_shooter.setControl(velocityRequest.withVelocity(rpm).withSlot(0));
        }
    }

    public Command shoot(double rpm) {
        return Commands.startEnd(() -> this.setShooterRPM(rpm), () -> this.setShooterRPM(0), this);
    }

    public void stop() {
        setRPM(0);
    }

    public void setHoodAngle(int pulseWidth) {
        m_hoodServoPulseWidth = Utils.clamp(pulseWidth, 1000, 2000);
        m_hoodServo.setPulseWidth(m_hoodServoPulseWidth);
    }

    public Command setHoodAngleCommand(int pulseWidth) {
        return Commands.runOnce(() -> setHoodAngle(pulseWidth), this);
    }

    private void updateHoodAngle() {
        m_hoodServo.setPulseWidth(m_hoodServoPulseWidth);
    }

    public void periodic() {
        setShooterRPM(m_speed);
        m_hoodServoPulseWidth = (int) SmartDashboard.getNumber("Hood Pulse Request", 1000);
        updateHoodAngle();
        SmartDashboard.putNumber("Shooter Velocity", m_shooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooting Value", m_speed);
        SmartDashboard.putNumber("Hood Pulse Width", m_hoodServo.getPulseWidth());
        SmartDashboard.putNumber("Requested PID", m_shooter.getMotorVoltage().getValueAsDouble());
    }

    public boolean atSetPoint() {
        return Math.abs(m_shooter.getVelocity().getValueAsDouble() - m_speed) < 1; // TODO: Adjust threshold as needed
    }
}
