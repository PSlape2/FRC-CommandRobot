package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoSetElevator extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;

    public AutoSetElevator() {
        elevatorSubsystem = new ElevatorSubsystem();
    }

    public void execute() {
        elevatorSubsystem.goTo(ElevatorConstants.kPredef_high);

        while(elevatorSubsystem.hasReached()) {}

        elevatorSubsystem.stopElevator();
    }

    public void end() {
        elevatorSubsystem.stopElevator();
    }
}
