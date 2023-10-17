package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax intake;

    public IntakeSubsystem() {
        intake = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);

        intake.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        intake.setIdleMode(IdleMode.kBrake);
    }
    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }
    public double getSpeed() {
        return intake.get();
    }
    public void stopIntake() {
        intake.stopMotor();
    }
}
