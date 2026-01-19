package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Wrist extends SubsystemBase {
    //
    // Hardware
    //

    private final SparkMax m_wristMotor = new SparkMax(14, MotorType.kBrushed);
    private final SparkAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder();
    private SparkClosedLoopController m_wristController = m_wristMotor.getClosedLoopController();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    //
    // State
    //
    private double m_Demand;


    static {
        double turningFactor = 2 * Math.PI;
        // TODO configure
        turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
    }

    public Wrist() {
        m_wristMotor.configure(turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_desiredState.angle = new Rotation2d(m_wristEncoder.getPosition());

    }

    public void stop() {
        m_Demand = 0.0;
    }

    public void setIntakeVoltage(double IntakeVoltage) {
        m_Demand = IntakeVoltage;
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // Command driving and turning SPARKS towards their respective setpoints.
        m_wristController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    @Override
    public void periodic() {
        m_wristMotor.set(m_Demand);
        SmartDashboard.putNumber("Wrist Position", m_wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Desired State", m_desiredState.angle.getRotations());
    }
}
