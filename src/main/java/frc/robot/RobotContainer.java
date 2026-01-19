// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator m_mechanism = new Elevator();
  //private final Intake m_intakMechanism = new Mechanism();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // m_driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // m_driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
    * Joystick Y = quasistatic forward
    * Joystick A = quasistatic reverse
    * Joystick B = dynamic forward
    * Joystick X = dyanmic reverse
    */
    m_driverController.y().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.a().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.b().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.x().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //SmartDashboard.putNumber("Arm test setpoint", 0);
    //m_driverController.leftTrigger().whileTrue(m_mechanism.armAngleCommand(()-> SmartDashboard.getNumber("Arm test setpoint", 0)));


    m_driverController.back().whileTrue(m_mechanism.homeCommand());
    m_driverController.povUp().whileTrue(m_mechanism.openLoopCommand(2));
    m_driverController.povDown().whileTrue(m_mechanism.openLoopCommand(-2));
    //m_driverController.rightBumper(m_intakMechanism.openLoopIntakeCommand(-50));
    //m_driverController.leftBumper(50);
    m_driverController.povLeft().whileTrue(m_mechanism.pidCommand(24));
    SmartDashboard.putNumber("Elevator test setpoint", 0);
    m_driverController.povRight().whileTrue(m_mechanism.pidCommand(()-> SmartDashboard.getNumber("Elevator test setpoint", 0)));
  }    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}