package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax rightShooter, leftShooter;
    private BangBangController shooterController;
    private RelativeEncoder leftEncoder, rightEncoder;
    private SimpleMotorFeedforward feedforward;

    public ShooterSubsystem() {
        rightShooter =  new CANSparkMax(ShooterConstants.shooterPorts[0], MotorType.kBrushless);
        leftShooter  = new CANSparkMax(ShooterConstants.shooterPorts[1], MotorType.kBrushless);

        rightShooter.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
        rightShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setInverted(false);

        leftShooter.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
        leftShooter.setIdleMode(IdleMode.kCoast);
        leftShooter.setInverted(true);

        rightEncoder = rightShooter.getEncoder();
        leftEncoder = leftShooter.getEncoder();
        shooterController = new BangBangController(ShooterConstants.kErrorTolerance);
        feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    }

    /**
     * Sets the speed of the shooter with feedforward
     * @param speed The target speed for the shooter
     */
    public void setSpeed(double speed) {
        rightShooter.setVoltage(
            shooterController.calculate(rightEncoder.getVelocity(), speed)
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