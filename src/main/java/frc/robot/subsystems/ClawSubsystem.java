package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private final CANSparkMax claw;
    public int gamePieceType;

    public ClawSubsystem() {
        claw = new CANSparkMax(ClawConstants.kPort, MotorType.kBrushless);
        claw.setSmartCurrentLimit(ClawConstants.kCurrentLimit);
        claw.setSecondaryCurrentLimit(ClawConstants.kCurrentLimit);
        claw.setIdleMode(IdleMode.kCoast);
        
        gamePieceType = 0;
    }

    /**
     * Gets the current game piece type (0 for cones, 1 for cubes)
     * @return The game piece type
     */
    public int getPieceType() {
        return gamePieceType;
    }

    /**
     * Switches the game piece type
     */
    public void switchGamePieceType() {
        if(gamePieceType == 0) {
            gamePieceType = 1;
        } else if(gamePieceType == 1) {
            gamePieceType = 0;
        } else {
            System.out.println("Game piece type not 1 or 0");
        }
    }

    /**
     * Sets the claw speed
     * @param speed The speed for the claw
     */
    public void setClaw(double speed) {
        claw.set(speed);
    }

    /**
     * Gets the claw current
     * @return Gets the output current of the claw
     */
    public double getCurrent() {
        return claw.getOutputCurrent();
    }

    /**
     * Gets the speed of the claw
     * @return The current speed of the claw
     */
    public double getSpeed() {
        return claw.get();
    }

    /**
     * Stops the claw
     */
    public void stopClaw() {
        claw.stopMotor();
    }

    /**
     * Does nothing
     */
    @Override
    public void periodic() {}
}
