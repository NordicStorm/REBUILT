package frc.robot.subsystems;

    import java.util.function.DoubleSupplier;

    import edu.wpi.first.wpilibj.Joystick;
    import edu.wpi.first.wpilibj.TimedRobot;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.FunctionalCommand;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

    import com.revrobotics.spark.SparkMax;
    import com.revrobotics.spark.SparkLowLevel.MotorType;
    import com.revrobotics.spark.SparkAbsoluteEncoder;
    import com.revrobotics.spark.SparkClosedLoopController;
    import com.revrobotics.spark.config.SparkMaxConfig;
    import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CIntake extends SubsystemBase{
    //
    // Hardware
    //
    private final SparkMax m_rightIntakeMotor = new SparkMax(17, MotorType.kBrushless);
    private final SparkMax m_leftIntakeMotor = new SparkMax(16,MotorType.kBrushless);
    private final SparkMax m_wristMotor = new SparkMax(4, MotorType.kBrushed);
    private final SparkAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder();
    //private SparkClosedLoopController m_rightIntakeController = m_rightIntakeMotor.getClosedLoopController();
    //private SparkClosedLoopController m_leftIntakeController = m_leftIntakeMotor.getClosedLoopController();
    private SparkClosedLoopController m_wristController = m_wristMotor.getClosedLoopController();
    private SparkMaxConfig m_rightIntakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_leftIntakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_wristMotorConfig = new SparkMaxConfig();

    //
    // State
    //
    private double m_intakeDemand;

    public CIntake() {
        m_rightIntakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20)
            .follow(3);
        m_leftIntakeMotorConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);       
    }

    public void stop() {
        m_intakeDemand = 0.0;
    }

    public void setIntakeVoltage(double IntakeVoltage) {
        m_intakeDemand = IntakeVoltage;
    }

    public Command openLoopIntakeCommand(DoubleSupplier IntakeVoltageSupplier) {
        return Commands.runEnd(
            () -> this.setIntakeVoltage(IntakeVoltageSupplier.getAsDouble()), this::stop, this);
    }

    public Command openLoopIntakeCommand(double IntakeVoltage) {
        return openLoopIntakeCommand(() -> IntakeVoltage);
    }

    @Override
    public void periodic() {
        m_leftIntakeMotor.set(m_intakeDemand);

    }    
}
