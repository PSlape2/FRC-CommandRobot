package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public int getPieceType() {
        return gamePieceType;
    }

    public void switchGamePieceType() {
        if(gamePieceType == 0) {
            gamePieceType = 1;
        } else if(gamePieceType == 1) {
            gamePieceType = 0;
        } else {
            System.out.println("Game piece type not 1 or 0");
        }
    }

    public void setClaw(double speed) {
        claw.set(speed);
    }

    public double getCurrent() {
        return claw.getOutputCurrent();
    }

    public double getSpeed() {
        return claw.get();
    }

    public void stopClaw() {
        claw.stopMotor();
    }

    @Override
    public void periodic() {}
}
