package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    //
    // Hardware
    //
    private final TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration m_intakePivotConfig = new TalonFXConfiguration();
    private final TalonFX m_intake = new TalonFX(Constants.MechanismConstants.kIntakeMotorID, "rio");
    private final TalonFX m_intakePivot = new TalonFX(Constants.MechanismConstants.kIntakePivotID, "rio");
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    
    public Intake() {

        var intakeSlot0Config = new Slot0Configs();
        intakeSlot0Config.kS = 0.1; // Add 0.1 V output to overcome static friction
        intakeSlot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        intakeSlot0Config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        intakeSlot0Config.kI = 0; // no output for integrated error
        intakeSlot0Config.kD = 0; // no output for error derivative

        m_intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_intakeConfig.CurrentLimits.SupplyCurrentLimit = 80;
        m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_intake.getConfigurator().apply(m_intakeConfig);
        m_intake.getConfigurator().apply(intakeSlot0Config);

        m_intake.optimizeBusUtilization();

        var intakePivotSlot0Config = new Slot0Configs();
        intakePivotSlot0Config.kS = 0.1; // Add 0.1 V output to overcome static friction
        intakePivotSlot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        intakePivotSlot0Config.kP = 0.11; // An error of 1 rps results in 0.11 V output
        intakePivotSlot0Config.kI = 0; // no output for integrated error
        intakePivotSlot0Config.kD = 0; // no output for error derivative

        m_intakePivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_intakePivotConfig.CurrentLimits.SupplyCurrentLimit = 80;
        m_intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_intakePivot.getConfigurator().apply(m_intakePivotConfig);
        m_intakePivot.getConfigurator().apply(intakePivotSlot0Config);

        m_intakePivot.optimizeBusUtilization();
    }
}
