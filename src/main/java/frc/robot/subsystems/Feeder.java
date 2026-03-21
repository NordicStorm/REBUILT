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
import frc.robot.Constants.FeederConstants;


public class Feeder extends SubsystemBase {

    private double m_speed = 0;

    private final TalonFXConfiguration m_feederConfig = new TalonFXConfiguration();
    public final TalonFX m_feeder = new TalonFX(Constants.MechanismConstants.kFeederMotorID, "rio");
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    final DutyCycleOut stopMotorRequest = new DutyCycleOut(0);

    public Feeder() {
        SmartDashboard.putNumber("Feeder RPS Request", 0);

        var feederSlot0Configs = new Slot0Configs();
        feederSlot0Configs.kV = FeederConstants.kV; 
        feederSlot0Configs.kP = FeederConstants.kP; 

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
        return Commands.startEnd(() -> this.setRPM(rpm), () -> this.setRPM(0), this);
    }

    public Command startFeed(double rpm) {
        return Commands.run(() -> this.setRPM(rpm), this);
    }

    public Command endFeed() {
        return Commands.run(() -> this.stop(), this);
    }

    public void stop() {
        setRPM(0);
    }

    @Override
    public void periodic() {
        setFeederRPM(m_speed);
        SmartDashboard.putNumber("Feeder Velocity", m_feeder.getVelocity().getValueAsDouble());
    }
}
