package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax[] motors;
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
    
        motors = new CANSparkMax[4];
        for(int i = 0; i < motors.length-1; i++) {
            motors[i] = new CANSparkMax(DriveConstants.motorPorts[i], MotorType.kBrushless);
            motors[i].restoreFactoryDefaults();
            motors[i].setSmartCurrentLimit(DriveConstants.kCurrentLimit);
            motors[i].setIdleMode(IdleMode.kBrake);
        }

        right = new MotorControllerGroup(new CANSparkMax[] {motors[0], motors[1]});
        left = new MotorControllerGroup(new CANSparkMax[] {motors[2], motors[3]});
        right.setInverted(true);
        differentialDrive = new DifferentialDrive(left, right);

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
    public void tankDrive(double l_speed, double r_speed) {
        differentialDrive.tankDrive(l_speed, r_speed);
    }
    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed, rotation);
    }
    public void tankDriveVolts(double leftVolt, double rightVolt) {
        left.setVoltage(leftVolt);
        right.setVoltage(rightVolt);
        differentialDrive.feed();
    }

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

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return wheelSpeeds;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return driveKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        targetWheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
    }

    public void resetOdometry(Pose2d newPose) {
        resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            newPose
        );
    }
    public void resetOdometry() {
        resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            new Pose2d()
        );
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getAverageDistance() {
        return 
            (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    public void setMaxOut(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    public double getAverageRate() {
        return (leftEncoder.getRate() + rightEncoder.getRate()) / 2.0;
    }
}
