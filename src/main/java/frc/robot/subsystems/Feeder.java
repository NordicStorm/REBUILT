package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Feeder extends SubsystemBase {

    private double m_speed = 0;

    private final TalonFXConfiguration m_feederConfig = new TalonFXConfiguration();
    public final TalonFX m_feeder = new TalonFX(Constants.MechanismConstants.kFeederMotorID, "rio");
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    final DutyCycleOut stopMotorRequest = new DutyCycleOut(0);



    public Feeder() {
        SmartDashboard.putNumber("Feeder RPS Request", 0);

        var feederSlot0Configs = new Slot0Configs();
        feederSlot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        feederSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        feederSlot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        feederSlot0Configs.kI = 0; // no output for integrated error
        feederSlot0Configs.kD = 0; // no output for error derivative

        m_feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_feederConfig.CurrentLimits.SupplyCurrentLimit = 80;
        m_feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_feeder.getConfigurator().apply(m_feederConfig);
        m_feeder.getConfigurator().apply(feederSlot0Configs);

        m_feeder.optimizeBusUtilization();
    }

    public void setRPM(double RPM) {
        m_speed = RPM / 60;
    }

    private void setFeederRPM(double rpm) {
        if (rpm == 0) {
            m_feeder.set(rpm);
        } else {
            m_feeder.setControl(velocityRequest.withVelocity(rpm));
        }
    }

    public Command feed(double rpm) {
        return Commands.startEnd(() -> this.setFeederRPM(rpm), () -> this.setFeederRPM(0), this);
    }

    public void stop() {
        setRPM(0);
    }

    public void setVoltage(double v) {
        m_feeder.setVoltage(v);
    }

    @Override
    public void periodic() {
        setFeederRPM(m_speed);
        SmartDashboard.putNumber("Feeder Velocity", m_feeder.getVelocity().getValueAsDouble());
    }
}
