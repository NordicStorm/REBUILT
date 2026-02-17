package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {

    private enum ControlMode {
        kStop, kPID
    }

    private final TalonFXConfiguration m_feederConfig = new TalonFXConfiguration();
    private final TalonFX m_feeder = new TalonFX(MechanismConstants.kFeederMotorID, "CANivore");

    public Feeder() {
        var feederSlot0Configs = new Slot0Configs();
        feederSlot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        feederSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        feederSlot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        feederSlot0Configs.kI = 0; // no output for integrated error
        feederSlot0Configs.kD = 0; // no output for error derivative

        m_feeder.getConfigurator().apply(feederSlot0Configs);

        m_feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_feederConfig.CurrentLimits.SupplyCurrentLimit = 80;

        m_feeder.getConfigurator().apply(m_feederConfig);
        m_feeder.optimizeBusUtilization();
    }

    private void setFeederRPM(double rpm) {
        m_feeder.setControl(new VelocityVoltage(rpm).withSlot(0));
    }

    public Command feed(double rpm) {
        return Commands.startEnd(() -> this.setFeederRPM(rpm), () -> this.setFeederRPM(0), this);
    }

    public void stop() {
        setFeederRPM(0);
    }

    @Override
    public void periodic() {
        //m_shooter.set(m_speed);
        SmartDashboard.putNumber("Velocity from Encoder", m_feeder.getVelocity().getValueAsDouble());
    }
}
