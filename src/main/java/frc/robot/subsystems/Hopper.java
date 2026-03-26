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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;;



public class Hopper extends SubsystemBase {

    private final TalonFXConfiguration m_hopperConfig = new TalonFXConfiguration();
    public final TalonFX m_hopper = new TalonFX(Constants.MechanismConstants.kHopperMotorID, "rio");
    final VelocityVoltage velocityRequest = new VelocityVoltage(-20);
    final DutyCycleOut stopMotorRequest = new DutyCycleOut(0);

    private boolean isFeeding = false;

    public Hopper() {
        SmartDashboard.putNumber("Hopper RPS Request", -20);

        var hopperSlot0Configs = new Slot0Configs();
        hopperSlot0Configs.kV = HopperConstants.kV; 
        hopperSlot0Configs.kP = HopperConstants.kP;

        m_hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_hopperConfig.CurrentLimits.SupplyCurrentLimit = 40;
        m_hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_hopper.getConfigurator().apply(m_hopperConfig);
        m_hopper.getConfigurator().apply(hopperSlot0Configs);

        m_hopper.optimizeBusUtilization();
    }

    private void setHopper(boolean isFeeding) {
        if (isFeeding) {
            double velocity = SmartDashboard.getNumber("Hopper RPS Request", 0);
            m_hopper.setControl(velocityRequest.withVelocity(velocity));
        } else {
            m_hopper.set(0);
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
        setHopper(isFeeding);
        SmartDashboard.putBoolean("Is feeding", isFeeding);
        SmartDashboard.putNumber("Hopper Velocity", m_hopper.getVelocity().getValueAsDouble());
    }
}
