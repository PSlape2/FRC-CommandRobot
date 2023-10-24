package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;
    private final Supplier<Boolean> shouldIntake, switchDirection, gamePieceSupplier;
    
    private boolean direction, shouldRun;
    private int gamePieceType;

    public ClawCommand(
        ClawSubsystem clawSubsystem,
        Supplier<Boolean> shouldIntake,
        Supplier<Boolean> switchDirection,
        Supplier<Boolean> gamePieceSupplier
    ) {
        this.clawSubsystem = clawSubsystem;
        this.shouldIntake = shouldIntake;
        this.switchDirection = switchDirection;
        this.gamePieceSupplier = gamePieceSupplier;

        gamePieceType = 0;
        direction = true;
        shouldRun = false;

        addRequirements(clawSubsystem);
    }

    // Use get bumper released

    public void execute() {

        if(gamePieceSupplier.get()) {
            clawSubsystem.switchGamePieceType();
            gamePieceType = clawSubsystem.getPieceType();
        }

        if(shouldIntake.get()) {
           shouldRun = true;
        } else {
            shouldRun = false;
        }

        if(switchDirection.get()) {
            direction = !direction;
        }

        if(shouldRun) {
            if(direction) {
                if(gamePieceType == 0) {                        // Cone
                    clawSubsystem.setClaw(ClawConstants.kSpeed);
                } else if(gamePieceType == 1) {                 // Cube
                    clawSubsystem.setClaw(-ClawConstants.kSpeed);
                }
            } else {
                if(gamePieceType == 0) {                        // Cone reversed
                    clawSubsystem.setClaw(-ClawConstants.kSpeed);
                } else if(gamePieceType == 1) {                 // Cube reversed
                    clawSubsystem.setClaw(ClawConstants.kSpeed);
                }
            }
        } else {
            clawSubsystem.stopClaw();
        }

        setDashboard();
    }


    private void setDashboard() {
        SmartDashboard.putBoolean("Claw Direction", direction);
        SmartDashboard.putNumber("Game Piece Type (0 for cone, 1 for cube)", gamePieceType);
        SmartDashboard.putNumber("Claw Forward Speed", ClawConstants.kSpeed);
    }

    @Override
        public void end(boolean interrupted) {
            clawSubsystem.stopClaw();
        }
}
