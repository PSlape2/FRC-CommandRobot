package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax[] motors;
    private final MotorControllerGroup right, left;
    private final DifferentialDrive differentialDrive;
    
    private final Encoder leftEncoder = new Encoder(
        DriveConstants.kLeftEncoderPorts[0],
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderReversed
    );

    private final Encoder rightEncoder = new Encoder(
        DriveConstants.kRightEncoderPorts[0],
        DriveConstants.kRightEncoderPorts[1],
        DriveConstants.kRightEncoderReversed
    );

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    DifferentialDriveKinematics driveKinematics =
            new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

    private final DifferentialDriveOdometry odometry;

    private DifferentialDriveWheelSpeeds wheelSpeeds, targetWheelSpeeds;

    public DriveSubsystem() {
        motors = new CANSparkMax[6];
        for(int i = 0; i < motors.length; i++) {
            motors[i] = new CANSparkMax(DriveConstants.motorPorts[i], MotorType.kBrushless);
            motors[i].setSmartCurrentLimit(DriveConstants.kCurrentLimit);
            motors[i].setSecondaryCurrentLimit(DriveConstants.kCurrentLimit);
            motors[i].setIdleMode(IdleMode.kBrake);
        }

        right = new MotorControllerGroup(motors[0], motors[1], motors[2]);
        left = new MotorControllerGroup(motors[3], motors[4], motors[5]);
        right.setInverted(true);

        differentialDrive = new DifferentialDrive(left, right);

        differentialDrive.setSafetyEnabled(false);

        leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

        odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(), 
            leftEncoder.getDistance(),
            rightEncoder.getDistance()
        );

         AutoBuilder.configureRamsete(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new ReplanningConfig(),
            this
         );
    }

    /**
     * Drives the roboot in a tank drive style
     * 
     * @param l_speed The speed for the left motor group
     * @param r_speed The speed for the right motor group
     */
    public void tankDrive(double l_speed, double r_speed) {
        differentialDrive.tankDrive(l_speed, r_speed);
    }

    /**
     * Drives the robot in an arcade drive style
     * 
     * @param speed The forward velocity of the robot
     * @param rotation The rotation of the robot
     */
    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed, rotation);
    }

    /**
     * Drives the robot in a tank drive style, but with voltage control
     * 
     * @param leftVolt The number of volts for the left motors
     * @param rightVolt The number of volts for the right motors
     */
    public void tankDriveVolts(double leftVolt, double rightVolt) {
        left.setVoltage(leftVolt);
        right.setVoltage(rightVolt);
        differentialDrive.feed();
    }

    /**
     * Updates the odometry and wheel speeds for the robot
     */
    @Override
    public void periodic() {
        odometry.update(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance()
        );

        wheelSpeeds = new DifferentialDriveWheelSpeeds(
            leftEncoder.getRate(),
            rightEncoder.getRate()
        );
    }

    /**
     * Gets the pose of the robot
     * @return The Pose2d of the robot from odometry
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets the wheel speeds
     * @return The DifferentialWheelSpeeds with the drive subsystem
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return wheelSpeeds;
    }

    /**
     * Translates the wheel speeds into chassis speeds
     * @return The chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return driveKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    /**
     * Sets and moves based on the chassis speeds
     * @param chassisSpeeds The chassisSpeeds to set to
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        targetWheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
        left.set(targetWheelSpeeds.leftMetersPerSecond);
        right.set(targetWheelSpeeds.rightMetersPerSecond);
    }

    /**
     * Resets the odometry to a new pose
     * @param newPose The pose to reset to
     */
    public void resetOdometry(Pose2d newPose) {
        resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            newPose
        );
    }

    /**
     * Resets the odometry completely
     */
    public void resetOdometry() {
        resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            new Pose2d()
        );
    }

    /**
     * Resets the encoders
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Gets the average distance traveled by the encoders
     * @return The average distance
     */
    public double getAverageDistance() {
        return 
            (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Sets the max output
     * @param maxOutput The max output
     */
    public void setMaxOut(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the gyro heading
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Gets the heading of the robot from the gyro
     * @return The heading of the robot
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Gets the turn rate of the robot
     * @return The turn rate
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
     * Gets the average speed of each encoder
     * @return The average speed
     */
    public double getAverageRate() {
        return (leftEncoder.getRate() + rightEncoder.getRate()) / 2.0;
    }

    /**
     * Gets the auto command using AutoBuilder. Currently unused.
     * @return The command generated from a path.
     */
    public Command followAutoBuilderPathCommand() {
        return AutoBuilder.followPathWithEvents(
            PathPlannerPath.fromPathFile("Basic Drive Path")
        );
    }

    /**
     * Sets the auto command based on the input path
     * 
     * @param path The path to be used for auto.
     * @return The auto command to be used.
     */
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathWithEvents(
            new FollowPathRamsete(
                path,
                this::getPose,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                new ReplanningConfig(),
                this
            ),
            path,
            this::getPose
        );  
    }

    public Command getAutoCommand() {
        return new PathPlannerAuto("Basic Drive Auto");
    }
}
