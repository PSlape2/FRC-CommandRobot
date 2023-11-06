package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public final class AutoDriveCommand {
    public static Command getAutoCommand() {
        DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

        DifferentialDriveKinematics driveKinematics =
            new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

        DifferentialDriveVoltageConstraint autoVoltageConstraint = 
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    AutoConstants.kS,
                    AutoConstants.kV,
                    AutoConstants.kA
                ),
                driveKinematics,
                10
            );
            
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeed,
            AutoConstants.kMaxAccel
        ).setKinematics(driveKinematics).addConstraint(autoVoltageConstraint);

        Trajectory exampleTrajecory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1,1), new Translation2d(2,-1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config
            );

        RamseteCommand ramCommand =
            new RamseteCommand(
                exampleTrajecory,
                driveSubsystem::getPose,
                new RamseteController(AutoConstants.kRamseteB,AutoConstants.kRamseteZ),
                new SimpleMotorFeedforward(
                    AutoConstants.kS,
                    AutoConstants.kV,
                    AutoConstants.kA
                ),
                driveKinematics,
                driveSubsystem::getWheelSpeed, // This is a lambda
                new PIDController(AutoConstants.kDriveVelocity, 0, 0),
                new PIDController(AutoConstants.kDriveVelocity, 0, 0),
                driveSubsystem::tankDriveVolts, // This is a lambda
                driveSubsystem
            );
        driveSubsystem.resetOdometry(exampleTrajecory.getInitialPose());

        return ramCommand.andThen(() -> driveSubsystem.tankDriveVolts(0,0));
    }
}
