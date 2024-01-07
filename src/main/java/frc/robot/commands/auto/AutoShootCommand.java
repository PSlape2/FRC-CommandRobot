package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    public AutoShootCommand() {
        shooterSubsystem = new ShooterSubsystem();
    }

    public void execute() {
        shooterSubsystem.setSpeed(ShooterConstants.kShooterSpeed);
    }

    public void end() {
        shooterSubsystem.stopShooter();
    }
}
