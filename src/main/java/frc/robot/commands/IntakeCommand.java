package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> rotateIntake, reverseIntake;

    boolean direction, isActive;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> rotateIntake, Supplier<Boolean> reverseIntake) {
        this.intakeSubsystem = intakeSubsystem;
        this.rotateIntake = rotateIntake;
        this.reverseIntake = reverseIntake;

        direction = false;

        addRequirements(intakeSubsystem);
    } 
    public void execute() {
        if(rotateIntake.get()) {
            isActive = true;
            direction = true;
        } else if(reverseIntake.get()) {
            direction = false;
            isActive = true;
        } else {
            isActive = false;
        }

        if(isActive) {
            if(direction) {
                intakeSubsystem.setIntakeSpeed(IntakeConstants.kInSpeed);
            } else {
                intakeSubsystem.setIntakeSpeed(IntakeConstants.kOutSpeed);
            }
        } else {
            intakeSubsystem.stopIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
