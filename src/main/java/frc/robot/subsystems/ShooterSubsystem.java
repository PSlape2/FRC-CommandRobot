package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.BangBangController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax rightShooter, leftShooter;
    private BangBangController shooterController;
    private RelativeEncoder leftEncoder, rightEncoder;
    private SimpleMotorFeedForward feedforward;

    public void execute() {
        rightShooter =  new CANSparkMax(ShooterConstants.motorPorts[0], MotorType.kBrushless);
        leftShooter  = new CANSparkMax(ShooterConstants.motorPorts[1], MotorType.kBrushless);

        rightShooter.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
        rightShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setInverted(false);

        leftShooter.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
        leftShooter.setIdleMode(IdleMode.kCoast);
        leftShooter.setInverted(true);

        rightEncoder = rightShooter.getEncoder();
        leftEncoder = leftShooter.getEncoder();
        shooterController = new BangBangController(ShooterConstants.kErrorTolerance);
        feedforward = new SimpleMotorFeedForward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    }
    public void setSpeed(double speed) {
        rightShooter.setVoltage(
            shooterController.calculate(rightEncoder.getVelocity(), speed))
                + 0.9 * feedforward.calculate(speed)
        );

        leftShooter.setVoltage(
            shooterController.calculate(leftEncoder.getVelocity(), speed)
                + 0.9 * feedforward.calculate(speed)
        );
    }

    public void stopShooter() {
        rightShooter.stopMotor();
        leftShooter.stopMotor();
    }
}