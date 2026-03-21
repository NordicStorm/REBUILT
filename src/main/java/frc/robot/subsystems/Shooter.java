package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MechanismConstants;

import frc.robot.Util;

public class Shooter extends SubsystemBase {

    //
    // Hardware
    //
    private final TalonFXConfiguration m_shooterConfig = new TalonFXConfiguration();
    public final TalonFX m_shooterLeft = new TalonFX(MechanismConstants.kLeftShooterID, "rio");
    public final TalonFX m_shooterMiddle = new TalonFX(MechanismConstants.kMiddleShooterID, "rio");
    public final TalonFX m_shooterRight = new TalonFX(MechanismConstants.kLeftShooterID, "rio");

    private final ServoHub m_ServoHub = new ServoHub(MechanismConstants.kServoHubID);
    private final ServoHubConfig m_servoHubConfig = new ServoHubConfig();
    private final ServoChannel m_leftHoodServo = m_ServoHub.getServoChannel(ChannelId.kChannelId1);
    private final ServoChannel m_rightHoodServo = m_ServoHub.getServoChannel(ChannelId.kChannelId2);

    final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private double m_speed = 0;
    private int m_hoodServoPulseWidth = 1000; // TODO

    //
    // PID
    //

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPS Request", 0);
        SmartDashboard.putNumber("Hood Pulse Request", 1000);

        var shooterSlot0Configs = new Slot0Configs();
        shooterSlot0Configs.kV = ShooterConstants.kF; 
        shooterSlot0Configs.kP = ShooterConstants.kP;
        shooterSlot0Configs.kI = ShooterConstants.kI;
        shooterSlot0Configs.kD = ShooterConstants.kD; 

        m_shooterConfig.Slot0 = shooterSlot0Configs;

        m_shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_shooterConfig.CurrentLimits.SupplyCurrentLimit = 80;
        m_shooterConfig.Slot0 = shooterSlot0Configs;

        m_shooterLeft.getConfigurator().apply(m_shooterConfig);
        m_shooterRight.getConfigurator().apply(m_shooterConfig);
        m_shooterMiddle.getConfigurator().apply(m_shooterConfig);


        m_shooterLeft.optimizeBusUtilization();
        m_shooterRight.optimizeBusUtilization();
        m_shooterMiddle.optimizeBusUtilization();


        m_servoHubConfig.channel0
                .pulseRange(1000, 1500, 2000)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        m_ServoHub.configure(m_servoHubConfig, ServoHub.ResetMode.kResetSafeParameters);
        m_rightHoodServo.setPowered(true);
        m_leftHoodServo.setEnabled(true);

        m_ServoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 10000);
    }

    public void setRPM(double RPM) {
        m_speed = RPM / 60;
    }

    private void setShooterRPM(double rpm) {
        if (rpm == 0) {
            m_shooterLeft.set(0);
            m_shooterMiddle.set(0);
            m_shooterRight.set(0);
        } else {
            m_shooterLeft.setControl(velocityRequest.withVelocity(rpm).withSlot(0));
        }
    }

    public Command shoot(double rpm) {
        return Commands.startEnd(() -> this.setShooterRPM(rpm), () -> this.setShooterRPM(0), this);
    }

    public void stop() {
        setRPM(0);
    }

    public void setHoodAngle(int pulseWidth) {
        m_hoodServoPulseWidth = (int) Util.clamp(pulseWidth, 1000, 2000);
        m_leftHoodServo.setPulseWidth(m_hoodServoPulseWidth);
        m_rightHoodServo.setPulseWidth(m_hoodServoPulseWidth);
    }

    public Command setHoodAngleCommand(int pulseWidth) {
        return Commands.runOnce(() -> setHoodAngle(pulseWidth), this);
    }

    private void updateHoodAngle() {
        m_leftHoodServo.setPulseWidth(m_hoodServoPulseWidth);
        m_rightHoodServo.setPulseWidth(m_hoodServoPulseWidth);
    }

    public void periodic() {
        setShooterRPM(m_speed);
        m_hoodServoPulseWidth = (int) SmartDashboard.getNumber("Hood Pulse Request", 1000);
        updateHoodAngle();
        SmartDashboard.putNumber("Shooter Velocity", m_shooterLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooting Value", m_speed);
        SmartDashboard.putNumber("Hood Pulse Width", m_rightHoodServo.getPulseWidth());
        SmartDashboard.putNumber("Requested PID", m_shooterLeft.getMotorVoltage().getValueAsDouble());
    }

    public boolean atSetPoint() {
        return Math.abs(m_shooterLeft.getVelocity().getValueAsDouble() - m_speed) < 1; // TODO: Adjust threshold as needed
    }
}
