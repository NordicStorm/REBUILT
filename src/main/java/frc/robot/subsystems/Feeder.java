package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

    private boolean isFeeding = false;

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

    private void setFeeder(boolean isFeeding) {
        if (isFeeding) {
            double velocity = SmartDashboard.getNumber("Feeder RPS Request", 0);
            m_feeder.setControl(velocityRequest.withVelocity(velocity));
        } else {
            m_feeder.set(0);
        }
    }

    public void setOn() {
        isFeeding = true;
    }

    public void setOff() {
        isFeeding = false;
    }

    @Override
    public void periodic() {
        setFeeder(isFeeding);
        SmartDashboard.putNumber("Feeder Velocity", m_feeder.getVelocity().getValueAsDouble());
    }
}
