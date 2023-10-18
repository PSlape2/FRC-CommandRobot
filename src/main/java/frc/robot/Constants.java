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
  }

  public static class IntakeConstants {
    public static final int kIntakePort = 3;
    public static final int kCurrentLimit = 45;

        // Constant speeds for intake
    public static final double kInSpeed = 0.5;
    public static final double kOutSpeed = -0.5;
  }
  public static class TankDriveConstants {
    public static final int[] motorPorts = {5, 6, 7, 8};
    public static final int kCurrentLimit = 45;

    public static final double kP = 0.5;    // Proportional constant
    public static final double kI = 0.5;    // Integral constant
    public static final double kD = 0.5;    // Derivative constant
    public static final double kMax = 1;    // Max PID output limit
    public static final double kMin = -1;   // Min PID output limit
  }
  public static class ElevatorConstants {
    public static final int kElevatorPort = 4;    // Elevator motor port
    public static final int kCurrentLimit = 45;   // Current limits

        // Constants for ElevatorFeedForward
    public static final double kS = 0.5;
    public static final double kG = 0.5;
    public static final double kV = 0.5;
    public static final double kA = 0;

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
}
