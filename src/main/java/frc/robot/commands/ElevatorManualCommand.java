package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.InputSystem;

/*
 * DO NOT USEL: Uses curved joystick input but does not have hard limits. May damage robot.
 */

public class ElevatorManualCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Double> elevatorSupplier;

    public ElevatorManualCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> elevatorSupplier) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSupplier = elevatorSupplier;
    }

    public void execute() {
        double inValue = elevatorSupplier.get();
        if(inValue > OperatorConstants.kOperatorElevatorDeadZone) {
            elevatorSubsystem.setElevatorSpeed(
                InputSystem.calculateInputWithCurve(
                    inValue, OperatorConstants.kOperatorElevatorCurve
                )
            );
        } else {
            elevatorSubsystem.stopElevator();
        }
    }
}
