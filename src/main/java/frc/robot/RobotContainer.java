// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  
  public final XboxController m_operatorController =
    new XboxController(OperatorConstants.kOperatorControllerPort);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final TankDriveSubsystem tankDriveSubsystem = new TankDriveSubsystem();

  private final  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    intakeSubsystem.setDefaultCommand(
      new IntakeCommand(
        intakeSubsystem, 
        () -> m_operatorController.getLeftBumperPressed(),
        () -> m_operatorController.getRightBumperPressed()
      )
    );

    elevatorSubsystem.setDefaultCommand(
      new ElevatorCommand(
        elevatorSubsystem,
        () -> m_operatorController.getAButtonPressed(),
        () -> m_operatorController.getXButtonPressed(),
        () -> m_operatorController.getYButtonPressed()
      )
    );

    tankDriveSubsystem.setDefaultCommand(
      new TankDriveCommand(
        tankDriveSubsystem,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightY()
      )
    );
    /*
    shooterSubsystem.setDefaultCommand(
      new ShooterCommand(
        shooterSubsystem,
        () -> m_operatorController.get
      );
    );
    */

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
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
