package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private Supplier<Boolean> shooterSupplier;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<Boolean> shooterSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSupplier = shooterSupplier;
        addRequirements(shooterSubsystem);
    }

    public void execute() {
        if(shooterSupplier.get()) {
            shooterSubsystem.set(ShooterConstants.kShooterSpeed);
        } else {
            shooterSubsystem.stopShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}