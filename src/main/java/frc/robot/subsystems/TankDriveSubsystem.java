package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankDriveConstants;

public class TankDriveSubsystem extends SubsystemBase {
    private CANSparkMax[] motors;
    private MotorControllerGroup right, left;
    private DifferentialDrive differentialDrive;
    
    public TankDriveSubsystem() {
    
        motors = new CANSparkMax[4];
        for(int i = 0; i < motors.length-1; i++) {
            motors[i] = new CANSparkMax(TankDriveConstants.motorPorts[i], MotorType.kBrushless);
            motors[i].restoreFactoryDefaults();
            motors[i].setSmartCurrentLimit(TankDriveConstants.kCurrentLimit);
            motors[i].setIdleMode(IdleMode.kBrake);
        }

        right = new MotorControllerGroup(new CANSparkMax[] {motors[0], motors[1]});
        left = new MotorControllerGroup(new CANSparkMax[] {motors[2], motors[3]});
        differentialDrive = new DifferentialDrive(left, right);
    }
    public void setSpeed(double l_speed, double r_speed) {
        differentialDrive.tankDrive(l_speed, r_speed);
    }
}
