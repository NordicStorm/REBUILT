package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hopper extends SubsystemBase {

    private double m_speed = 0;

    private final TalonFXConfiguration m_hopperConfig = new TalonFXConfiguration();
    public final TalonFX m_hopper = new TalonFX(Constants.MechanismConstants.kHopperMotorID, "rio");
    final VelocityVoltage velocityRequest = new VelocityVoltage(-15);
    final DutyCycleOut stopMotorRequest = new DutyCycleOut(0);



    public Hopper() {
        SmartDashboard.putNumber("Hopper RPS Request", 0);

        var hopperSlot0Configs = new Slot0Configs();
        hopperSlot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        hopperSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        hopperSlot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        hopperSlot0Configs.kI = 0; // no output for integrated error
        hopperSlot0Configs.kD = 0; // no output for error derivative

        m_hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_hopperConfig.CurrentLimits.SupplyCurrentLimit = 80;
        m_hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_hopper.getConfigurator().apply(m_hopperConfig);
        m_hopper.getConfigurator().apply(hopperSlot0Configs);

        m_hopper.optimizeBusUtilization();
    }

    public void setRPM(double RPM) {
        m_speed = RPM / 60;
    }

    private void setHopperRPM(double rpm) {
        if (rpm == 0) {
            m_hopper.set(rpm);
        } else {
            m_hopper.setControl(velocityRequest.withVelocity(rpm));
        }
    }

    public Command feed(double rpm) {
        return Commands.startEnd(() -> this.setHopperRPM(rpm), () -> this.setHopperRPM(0), this);
    }

    public void stop() {
        setRPM(0);
    }

    public void setVoltage(double v) {
        m_hopper.setVoltage(v);
    }

    @Override
    public void periodic() {
        setHopperRPM(m_speed);
        SmartDashboard.putNumber("Hopper Velocity", m_hopper.getVelocity().getValueAsDouble());
    }
}
