package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {

    //
    // Hardware
    //
    private final TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration m_intakePivotConfig = new TalonFXConfiguration();
    private final TalonFX m_intake = new TalonFX(MechanismConstants.kIntakeMotorID, "rio");
    private final TalonFX m_intakePivot = new TalonFX(MechanismConstants.kIntakePivotID, "rio");
    private final PositionVoltage upPositionRequest = new PositionVoltage(-.03);
    private final PositionVoltage downPositionRequest = new PositionVoltage(-.35);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    final DutyCycleOut stopMotorRequest = new DutyCycleOut(0);

    private double m_speed = 0;
    private boolean setIntakeUp = true;

    public Intake() {
        SmartDashboard.putNumber("Intake RPS Request", 0);

        var intakeSlot0Config = new Slot0Configs();
        intakeSlot0Config.kS = IntakeConstants.kS_Intake; 
        intakeSlot0Config.kP = IntakeConstants.kP_Intake;
        intakeSlot0Config.kI = IntakeConstants.kI_Intake;
        intakeSlot0Config.kD = IntakeConstants.kD_Intake;

        m_intakeConfig
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(90))
                                .withSupplyCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(intakeSlot0Config);

        m_intake.getConfigurator().apply(m_intakeConfig);

        m_intake.optimizeBusUtilization();

        var intakePivotSlot1Config = new Slot1Configs(); // Down motion config
        intakePivotSlot1Config.kP = IntakeConstants.kP_Pivot_Down;
        intakePivotSlot1Config.kD = IntakeConstants.kD_Pivot_Down;
        intakePivotSlot1Config.withGravityType(GravityTypeValue.Arm_Cosine);
        intakePivotSlot1Config.withGravityArmPositionOffset(.25);

        m_intakePivotConfig
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(80))
                                .withSupplyCurrentLimit(Amps.of(30))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                .withSlot1(intakePivotSlot1Config)
                .Feedback.withSensorToMechanismRatio(25);

        m_intakePivot.optimizeBusUtilization();

        m_intakePivot.getConfigurator().apply(m_intakePivotConfig);
    }

    public Command moveIntake(boolean up) {
        // move intake up or down when command starts, always move it down when command
        // ends
        return Commands.startEnd(() -> this.setIntakePivotPosition(up), () -> this.setIntakePivotPosition(false), this);
    }

    public void switchPosition() {
        setIntakeUp = !setIntakeUp;
    }

    public void setIntakeUp() {
        setIntakeUp = true;
    }

    public void setIntakeDown() {
        setIntakeUp = false;
    }

    // Returns true if the intake is up or is in the process of moving up.
    public boolean isIntakeUp() {
        return setIntakeUp;
    }

    private double getIntakePivotLocation() {
        return m_intakePivot.getPosition().getValueAsDouble();
    }

    private void setIntakePivotPosition(boolean up) {
        m_intakePivot.setControl(up ? upPositionRequest.withSlot(1) : downPositionRequest.withSlot(1)); // First should be 0
    }

    public void setRPM(double RPM) {
        m_speed = RPM / 60;
    }

    public boolean atSetPoint() {
        double currentPosition = m_intakePivot.getPosition().getValueAsDouble();
        double targetPosition = setIntakeUp ? upPositionRequest.getPositionMeasure().magnitude()
                : downPositionRequest.getPositionMeasure().magnitude();
        return Math.abs(currentPosition - targetPosition) < .1; // Tolerance of .1 rot
    }

    public Command intake(double rpm) {
        return Commands.startEnd(() -> this.setIntakeRPM(rpm), () -> this.setIntakeRPM(0), this);
    }

    public void stop() {
        setRPM(0);
    }

    private void setIntakeRPM(double rpm) {
        if (rpm == 0) {
            m_intake.set(rpm);
        } else {
            m_intake.setControl(velocityRequest.withVelocity(rpm));
        }
    }

    @Override
    public void periodic() {
        setIntakeRPM(m_speed);
        setIntakePivotPosition(setIntakeUp);
        SmartDashboard.putNumber("Intake Position Request", setIntakeUp ? 0 : -.3);
        SmartDashboard.putNumber("Intake Pivot Position", getIntakePivotLocation());
    }
}
