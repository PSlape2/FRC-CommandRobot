package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TankDriveSubsystem;

public class TankDriveCommand extends CommandBase {
    private final TankDriveSubsystem tankDriveSubsystem;
    private final Supplier<Double> leftInput, rightInput;
    
    public TankDriveCommand(
        TankDriveSubsystem tankDriveSubsystem, 
        Supplier<Double> leftInput, Supplier<Double> rightInput
    ) {
        this.tankDriveSubsystem = tankDriveSubsystem;
        this.leftInput = leftInput;
        this.rightInput = rightInput;
    }
    
    public void execute() {
        double leftOut, rightOut;

        if(leftInput.get() > OperatorConstants.kDriverTankDriveDeadZone) {
            leftOut = processInput(leftInput.get());
        } else {
            leftOut = 0;
        }
        if(rightInput.get() > OperatorConstants.kDriverTankDriveDeadZone) {
            rightOut = processInput(rightInput.get());
        } else {
            rightOut = 0;
        }

        tankDriveSubsystem.setSpeed(leftOut, rightOut);
    }
    public double processInput(double in) {
        return OperatorConstants.kDriverTankDriveMax * Math.pow(in, OperatorConstants.kDriverTankDriveCurve);
    }
}
