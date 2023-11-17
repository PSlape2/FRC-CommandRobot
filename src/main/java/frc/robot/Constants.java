// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

        // Driver Constants
    public static final int kDriverControllerPort = 0;
    public static final double kDriverTankDriveDeadZone = 0.2;
    public static final double kDriverTankDriveMax = 0.7;
    public static final double kDriverTankDriveCurve = 1.8;

        // Operator Constants
    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorElevatorDeadZone = 0.2;
    public static final double kOperatorElevatorCurve = 0.5;
    public static final double kOperatorElevatorInitialValue = 0;
    public static final double kShooterDeadzone = 0.1;
  }

  public static class IntakeConstants {
    public static final int kIntakePort = 12;
    public static final int kCurrentLimit = 45;

        // Constant speeds for intake
    public static final double kInSpeed = 0.8;
    public static final double kOutSpeed = -0.8;
  }

  public static class DriveConstants {
    public static final int[] motorPorts = {3, 4, 5, 6, 9, 10};
    public static final int kCurrentLimit = 45;

        // Constants for PID Controller
    public static final double kP = 0.28;    // Proportional constant
    public static final double kI = 0.0;    // Integral constant
    public static final double kD = 0.055;    // Derivative constant
    public static final double kMax = 1;    // Max PID output limit
    public static final double kMin = -1;   // Min PID output limit
    
    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final int[] kRightEncoderPorts = {2, 3};
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;
    public static final double kDistancePerPulse = 0.5;

    public static final double kTrackWidth = 0.69; // In meters
  }

  public static class ElevatorConstants {
    public static final int kElevatorPort = 4;    // Elevator motor port
    public static final int kCurrentLimit = 45;   // Current limits

        // Constants for ElevatorFeedForward
    public static final double kS = 0.22; // Volts
    public static final double kV = 1.98; // Volt Seconds per Meter
    public static final double kG = 0.5;
    public static final double kA = 0.2; // Volt Seconds Squared per Meter

        // Constants for SparkMaxPIDController
    public static final double kP = 0.5;    // Proportional constant
    public static final double kI = 0.5;    // Integral constant
    public static final double kD = 0.5;    // Derivative constant
    public static final double kFF = 0.5;   // Feedforward constant
    public static final double kMax = 10;   // Max PID output limit (currently used for position)
    public static final double kMin = 0;    // Min PID output limit (currently used for position)

    public static final double kPredef_low = 10;
    public static final double kPredef_mid = 20;
    public static final double kPredef_high = 30;

    public static final double kErrorTolerance = 5;
    public static final double kDerivativeTolerance = 10;

    private static final double kSprocketRadius = 0.2;
    public static final double kPositionConversionFactor = 2 * Math.PI * kSprocketRadius * (1.0 / 8.0);
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; 
  }

  public static class ShooterConstants {
    public static final int[] shooterPorts = {1, 2};
    public static final double kErrorTolerance = 0.5;
    public static final int kCurrentLimit = 45;

    public static final double kS = 0; // Volts
    public static final double kV = 1; // Volt Seconds per Meter
    public static final double kA = 1; // Volt Seconds Squared per Meter
    
    public static final double kShooterSpeed = 0.6;
  }
  public static class ClawConstants {
    public static final int kPort = 22;
    public static final double kSpeed = 2;
    public static final int kCurrentLimit = 2;

  }
  public static class AutoConstants {
    public static final double kS = 0.22;
    public static final double kV = 1.98;
    public static final double kA = 0.2;

    public static final double kDriveVelocity = 8.5;
    public static final double kMaxSpeed = 3; // Meters per second
    public static final double kMaxAccel = 1; // Meters per second squared
    public static final double kRamseteB = 2; // Meters
    public static final double kRamseteZ = 0.7; // Seconds
  }
}
