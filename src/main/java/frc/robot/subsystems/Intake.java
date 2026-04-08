package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
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
    private final PositionVoltage upPositionRequest = new PositionVoltage(-.01);
    private final PositionVoltage downPositionRequest = new PositionVoltage(-.345);
    private final PositionVoltage agitatePositionRequest = new PositionVoltage(-.29).withSlot(2);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    final DutyCycleOut stopMotorRequest = new DutyCycleOut(0);

    private double m_speed = 0;
    private boolean setIntakeUp = true;
    private boolean agitate = false;
    private long agitateStartTime = 0;
    private long atHighCurrentSince = System.currentTimeMillis();

    public Intake() {
        SmartDashboard.putNumber("Intake RPS Request", 0);

        var intakeSlot0Config = new Slot0Configs();
        intakeSlot0Config.kS = IntakeConstants.kV_Intake;
        intakeSlot0Config.kP = IntakeConstants.kP_Intake;
        intakeSlot0Config.kI = IntakeConstants.kI_Intake;
        intakeSlot0Config.kD = IntakeConstants.kD_Intake;
        intakeSlot0Config.kV = .1;

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

        var intakePivotSlot0Config = new Slot0Configs();
        intakePivotSlot0Config.kP = IntakeConstants.kP_Pivot_Down;
        intakePivotSlot0Config.kD = IntakeConstants.kD_Pivot_Down;
        intakePivotSlot0Config.kG = IntakeConstants.kG_Pivot_Up;
        intakePivotSlot0Config.withGravityType(GravityTypeValue.Arm_Cosine);
        intakePivotSlot0Config.withGravityArmPositionOffset(.25);

        var intakePivotSlot1Config = new Slot1Configs(); // Down motion config
        intakePivotSlot1Config.kP = IntakeConstants.kP_Pivot_Down;
        intakePivotSlot1Config.kD = IntakeConstants.kD_Pivot_Down;
        intakePivotSlot1Config.withGravityType(GravityTypeValue.Arm_Cosine);
        intakePivotSlot1Config.withGravityArmPositionOffset(.25);

        var intakePivotSlot2Config = new Slot2Configs(); // Agitate motion config
        intakePivotSlot2Config.kP = IntakeConstants.kP_Agitate;
        intakePivotSlot2Config.kD = IntakeConstants.kD_Agitate;
        intakePivotSlot2Config.kG = IntakeConstants.kG_Agitate;
        intakePivotSlot2Config.withGravityType(GravityTypeValue.Arm_Cosine);
        intakePivotSlot2Config.withGravityArmPositionOffset(.25);

        m_intakePivotConfig
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(80))
                                .withSupplyCurrentLimit(Amps.of(40))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(intakePivotSlot0Config)
                .withSlot1(intakePivotSlot1Config)
                .withSlot2(intakePivotSlot2Config).Feedback.withSensorToMechanismRatio(25);

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
        m_intakePivot.setControl(up ? upPositionRequest.withSlot(0) : downPositionRequest.withSlot(1)); // First should
                                                                                                        // be 0
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

    public boolean isClear() {
        return m_intakePivot.getPosition().getValueAsDouble() < -.15;
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

    public boolean isStalled() {
        return System.currentTimeMillis() - atHighCurrentSince > 750;
    }

    @Override
    public void periodic() {
        if (this.isStalled() && false) {
            setIntakeRPM(-2000);
        } else if (agitate) {
            setIntakeRPM(1500);
        }
        else {
            setIntakeRPM(m_speed);
        }
        if (agitate && m_speed == 0) {
            long elapsedTime = System.currentTimeMillis() - agitateStartTime;
            if (elapsedTime < 2000) {
            } else {
                // Switch between forward and reverse every 1.5 seconds
                if ((elapsedTime / 1500) % 2 == 0) {
                    m_intakePivot.setControl(agitatePositionRequest.withPosition(-0.05));
                    SmartDashboard.putBoolean("Agitate is up", true);
                } else {
                    m_intakePivot.setControl(agitatePositionRequest.withPosition(-0.2));
                    SmartDashboard.putBoolean("Agitate is up", false);
                }
            }
        } else {
            setIntakePivotPosition(setIntakeUp);
        }

        if (m_intake.getSupplyCurrent().getValueAsDouble() < 50) {
            atHighCurrentSince = System.currentTimeMillis();
        }
        SmartDashboard.putNumber("Intake Position Request", setIntakeUp ? 0 : -.3);
        SmartDashboard.putNumber("Intake Pivot Position", getIntakePivotLocation());
    }

    public void setAgitateMode(boolean agitate) {
        this.agitate = agitate;
        if (agitate) {
            agitateStartTime = System.currentTimeMillis();
        }
    }
}
