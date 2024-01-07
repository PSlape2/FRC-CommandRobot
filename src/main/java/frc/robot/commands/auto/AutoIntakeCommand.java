package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    public AutoIntakeCommand() {
        intakeSubsystem = new IntakeSubsystem();
    }

    public void execute() {
        intakeSubsystem.setIntakeSpeed(IntakeConstants.kInSpeed);
    }

    public void end() {
        intakeSubsystem.stopIntake();
    }
}
