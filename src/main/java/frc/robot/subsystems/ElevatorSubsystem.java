package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax elevatorMotor;
    // private ElevatorFeedforward feedforward;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController pid;

    public ElevatorSubsystem() {
        elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorPort, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        pid = elevatorMotor.getPIDController();
        pid.setP(ElevatorConstants.kP);
        pid.setI(ElevatorConstants.kI);
        pid.setD(ElevatorConstants.kD);
        pid.setFF(ElevatorConstants.kFF);
        pid.setOutputRange(ElevatorConstants.kMin, ElevatorConstants.kMax);

        elevatorEncoder = elevatorMotor.getEncoder();
        /*
        
        feedforward = new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
        */
    }
    public void setElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
    }
    public double getElevatorSpeed() {
        return elevatorMotor.get();
    }
    public void stopElevator() {
        elevatorMotor.stopMotor();
    }
    public void setPIDTarget(double target) {
        pid.setReference(target, ControlType.kPosition);
    }
    public void setPIDTargetSpeed(double speed) {
        pid.setReference(speed, ControlType.kVelocity);
    }
    public void setPIDTargetVoltage(double voltage) {
        pid.setReference(voltage, ControlType.kVoltage);
    }
    public double getPosition() {
        return elevatorEncoder.getPosition();
    }
    public double getVelocity() {
        return elevatorEncoder.getVelocity();
    }
}
