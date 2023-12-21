package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private Supplier<Double> shooterSupplier;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<Double> shooterSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSupplier = shooterSupplier;
        addRequirements(shooterSubsystem);
    }

    public void execute() {
        if(Math.abs(shooterSupplier.get()) > OperatorConstants.kShooterDeadzone) {
            shooterSubsystem.setSpeed(ShooterConstants.kShooterSpeed);
        } else {
            shooterSubsystem.stopShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}