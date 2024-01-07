package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.auto.AutoSetElevator;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax elevatorMotor;
    private ElevatorFeedforward feedforward;
    private RelativeEncoder elevatorEncoder;
    private ProfiledPIDController elevatorController;
    private TrapezoidProfile.Constraints elevatorProfile;

    private double lastTime = Timer.getFPGATimestamp();
    private double lastSpeed = 0;
    
    public ElevatorSubsystem() {
        elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorPort, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ElevatorConstants.kPositionConversionFactor);
        elevatorEncoder.setVelocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

        elevatorProfile = new TrapezoidProfile.Constraints(ElevatorConstants.kMax, ElevatorConstants.kMin);

        elevatorController = new ProfiledPIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            elevatorProfile
        );

        elevatorController.setTolerance(
            ElevatorConstants.kErrorTolerance, ElevatorConstants.kDerivativeTolerance
        );
        
        feedforward = new ElevatorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kG,
            ElevatorConstants.kV,
            ElevatorConstants.kA
        );
    }

    /**
     * Sets the elevator to a specific position
     * @param pos The position to set to
     */
    public void goTo(double pos) {
        elevatorController.setGoal(pos);
        double acceleration = (elevatorController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

        elevatorMotor.setVoltage(
            elevatorController.calculate(elevatorEncoder.getVelocity(), pos)
             + feedforward.calculate(elevatorController.getSetpoint().velocity, acceleration)
        );

        lastSpeed = elevatorController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    public boolean hasReached() {
        return elevatorController.atGoal();
    }

    /**
     * Stops the elevator
     */
    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    /**
     * Gets the state of the TrapezoidProfile
     * @return The setpoint of the PID controller
     */
    public TrapezoidProfile.State getState() {
        return elevatorController.getSetpoint();
    }

    /**
     * Gets the position of the elevator
     * @return The position of the elevator
     */
    public double getPosition() {
        return elevatorController.getSetpoint().position;
    }

    /**
     * Gets the velocity of the elevator
     * @return The velocity of the elevator
     */
    public double getVelocity() {
        return elevatorController.getSetpoint().velocity;
    }

    public Command getAutoCommand() {
        return new AutoSetElevator();
    }
}
