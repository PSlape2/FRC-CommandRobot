package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TankDriveCommand extends CommandBase {
    private final DriveSubsystem tankDriveSubsystem;
    private final Supplier<Double> leftInput, rightInput;
    
    public TankDriveCommand(
        DriveSubsystem tankDriveSubsystem, 
        Supplier<Double> leftInput, Supplier<Double> rightInput
    ) {
        this.tankDriveSubsystem = tankDriveSubsystem;
        this.leftInput = leftInput;
        this.rightInput = rightInput;

        addRequirements(tankDriveSubsystem);
    }
    
    public void execute() {
        double leftOut, rightOut;
        int leftMult = -1;
        int rightMult = -1;

        if(Math.abs(leftInput.get()) > OperatorConstants.kDriverTankDriveDeadZone) {
            if(leftInput.get() >= 0) leftMult = -1;
            else leftMult = 1;

            leftOut = leftMult * processInput(Math.abs(leftInput.get()));
        } else {
            leftOut = 0;
        }
        if(Math.abs(rightInput.get()) > OperatorConstants.kDriverTankDriveDeadZone) {
            if(rightInput.get() >= 0) rightMult = -1;
            else rightMult = 1;

            rightOut = rightMult * processInput(Math.abs(rightInput.get()));
        } else {
            rightOut = 0;
        }

        tankDriveSubsystem.tankDrive(leftOut, rightOut);
    }
    public double processInput(double in) {
        return OperatorConstants.kDriverTankDriveMax * Math.pow(in, OperatorConstants.kDriverTankDriveCurve);
    }
}
