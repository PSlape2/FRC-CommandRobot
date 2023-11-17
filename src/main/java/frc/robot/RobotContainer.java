// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.auto.AutoDriveCommandContainer;
import frc.robot.subsystems.*;

import frc.robot.Constants.OperatorConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, IO devices, and commands. */

  public final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  
  public final XboxController m_operatorController =
    new XboxController(OperatorConstants.kOperatorControllerPort);

  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // private final ClawSubsystem clawSubsystem = new ClawSubsystem();

  // private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    /*
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    */

    /*
    intakeSubsystem.setDefaultCommand(
      new IntakeCommand(
        intakeSubsystem, 
        () -> m_operatorController.getLeftBumperPressed(),
        () -> m_operatorController.getRightBumperPressed()
      )
    );
    */
    /*
    elevatorSubsystem.setDefaultCommand(
      new ElevatorCommand(
        elevatorSubsystem,
        () -> m_operatorController.getAButtonPressed(),
        () -> m_operatorController.getXButtonPressed(),
        () -> m_operatorController.getYButtonPressed()
      )
    );
    */

    driveSubsystem.setDefaultCommand(
      new TankDriveCommand(
        driveSubsystem,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightY()
      )
    );
    
    shooterSubsystem.setDefaultCommand(
      new ShooterCommand(
        shooterSubsystem,
        () -> m_operatorController.getRightTriggerAxis()
      )
    );
    /*
    clawSubsystem.setDefaultCommand(
      new ClawCommand(
        clawSubsystem,
        () -> m_operatorController.getLeftBumperPressed(),
        () -> m_operatorController.getRightBumperReleased(),
        () -> m_operatorController.getBButtonReleased()
      )
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
  public Command getAutonomousCommand(PathPlannerPath path) {
    // return autoChooser.getSelected();
    return driveSubsystem.followPathCommand(path);
  }

}
