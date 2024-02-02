package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveToPose extends CommandBase {
    Pose2d pose;
    DriveSubsystem driveSubsystem;

    public AutoDriveToPose(Pose2d pose, DriveSubsystem drive) {
        this.pose = pose;
        driveSubsystem = drive;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        driveSubsystem.navigateToPose(pose);
    }
}
