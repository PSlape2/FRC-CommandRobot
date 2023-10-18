package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase{
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Boolean> lowPositionSupplier, midPositionSupplier, highPositionSupplier;
    
    public ElevatorCommand(
        ElevatorSubsystem elevatorSubsystem, Supplier<Boolean> lowPositionSupplier, 
        Supplier<Boolean> midPositionSupplier, Supplier<Boolean> highPositionSupplier
    ) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.lowPositionSupplier = lowPositionSupplier;
        this.midPositionSupplier = midPositionSupplier;
        this.highPositionSupplier = highPositionSupplier;
        addRequirements(elevatorSubsystem);
    } 
    
    public void execute() {
        if(lowPositionSupplier.get()) {
            elevatorSubsystem.goTo(ElevatorConstants.kPredef_low);
        } else if(midPositionSupplier.get()) {
            elevatorSubsystem.goTo(ElevatorConstants.kPredef_mid);
        } else if(highPositionSupplier.get()) {
            elevatorSubsystem.goTo(ElevatorConstants.kPredef_high);
        } else {
            elevatorSubsystem.stop();   
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }
}
