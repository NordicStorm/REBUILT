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
import frc.robot.RobotContainer;
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
    private Mode currentMode = Mode.OFF;
    private int m_hoodServoPulseWidth = 1100; // TODO

    public enum Mode {
        OFF,
        HUB,
        MANUAL,
        PASS
    }

    //
    // PID
    //

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPS Request", 0);
        SmartDashboard.putNumber("Hood Pulse Request", 1100);

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
        m_rightHoodServo.setEnabled(true);
        m_leftHoodServo.setPowered(true);
        m_leftHoodServo.setEnabled(true);

    }

    public void setManualRPS(double RPS) {
        m_speed = RPS;
        currentMode = Mode.MANUAL;
    }

    private void setShooterRPS(double rps) {
        if (rps == 0) {
            m_shooterLeft.set(0);
            m_shooterMiddle.set(0);
            m_shooterRight.set(0);
        } else {
            m_shooterLeft.setControl(velocityRequest.withVelocity(rps).withSlot(0));
            m_shooterLeft.setControl(velocityRequest.withVelocity(rps).withSlot(0));
            m_shooterLeft.setControl(velocityRequest.withVelocity(rps).withSlot(0));
        }
    }

    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void stop() {
        currentMode = Mode.OFF;
    }

    public void setHoodAngle(int pulseWidth) {
        m_hoodServoPulseWidth = (int) Util.clamp(pulseWidth, 1100, 1650);
        m_leftHoodServo.setPulseWidth(m_hoodServoPulseWidth);
        m_rightHoodServo.setPulseWidth(m_hoodServoPulseWidth);
    }

    public boolean atSetPoint() {
        return Math.abs(m_shooterLeft.getVelocity().getValueAsDouble() - m_speed) < 1 || m_speed == 0; // TODO: Adjust
                                                                                                       // threshold as
        // needed
    }

    private double getShootRPSFromDistance(double distance) {
        double x = distance;
        double result = -0.071*x*x + 0.529*x + 0.800; // CURVE:RPS,08:17,03/23
        return result;
    }

    private double getShootHoodFromDistance(double distance) {
        double x = distance;
        double result = -0.429*x*x + 3.171*x + -2.200; // CURVE:Hood,08:17,03/23
        return result;
    }

    private double getPassRPSFromDistance(double distance) {
        double x = distance;
        double result = -0.071*x*x + 0.529*x + 0.800; // CURVE:RPS,08:17,03/23
        return result;
    }

    private double getPassHoodFromDistance(double distance) {
        double x = distance;
        double result = -0.429*x*x + 3.171*x + -2.200; // CURVE:Hood,08:17,03/23
        return result;
    }

    public void periodic() {
        if (currentMode == Mode.HUB) {
            m_speed = getShootRPSFromDistance(RobotContainer.drivetrain.getDistanceToVirtualHub());
            m_hoodServoPulseWidth = (int) getShootHoodFromDistance(RobotContainer.drivetrain.getDistanceToVirtualHub());
        } else if (currentMode == Mode.PASS) {
            m_speed = getPassRPSFromDistance(RobotContainer.drivetrain.getDistanceToPassPoint());
            m_hoodServoPulseWidth = (int) getPassHoodFromDistance(RobotContainer.drivetrain.getDistanceToPassPoint());
        } else if (currentMode == Mode.MANUAL) {

        } else if (currentMode == Mode.OFF || currentMode == null) {
            m_speed = 0;
        }

        setShooterRPS(m_speed);
        setHoodAngle(m_hoodServoPulseWidth);

        SmartDashboard.putNumber("Recieved number", m_hoodServoPulseWidth);
        SmartDashboard.putNumber("Shooter Velocity", m_shooterLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooting Value", m_speed);
        SmartDashboard.putNumber("Hood Pulse Width", m_rightHoodServo.getPulseWidth());
        SmartDashboard.putNumber("Requested PID", m_shooterLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Distance to Hub", RobotContainer.drivetrain.getDistanceToVirtualHub());
    }
}