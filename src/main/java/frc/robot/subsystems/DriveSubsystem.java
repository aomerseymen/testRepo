/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */
  private final VictorSP frontLeftMotor = new VictorSP(Constants.DriveConstants.frontLeftMotorPin);
  private final VictorSP frontRightMotor = new VictorSP(Constants.DriveConstants.frontRightMotorPin);
  private final VictorSP rearLeftMotor = new VictorSP(Constants.DriveConstants.rearLeftMotorPin);
  private final VictorSP rearRightMotor = new VictorSP(Constants.DriveConstants.rearRightMotorPin);

  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, rearRightMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  private final Encoder driveEncoder = new Encoder(DriveConstants.driveEncoder_A, DriveConstants.driveEncoder_B, false,
      EncodingType.k4X);

  private final Encoder rightWheelEncoder = new Encoder(DriveConstants.rightWheelEncoder_A,
      DriveConstants.rightWheelEncoder_B, false, EncodingType.k4X);
  private final Encoder leftWheelEncoder = new Encoder(DriveConstants.leftWheelEncoder_A,
      DriveConstants.leftWheelEncoder_B, false, EncodingType.k4X);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public DriveSubsystem() {
    driveEncoder.setDistancePerPulse(7.62 * 2 * Math.PI / 2048.0);
    rightWheelEncoder.setDistancePerPulse(7.62 * 2 * Math.PI / 2048.0);
    leftWheelEncoder.setDistancePerPulse(7.62 * 2 * Math.PI / 2048.0);
    gyro.calibrate();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);

  }

  public double getDriveDistance() {
    return driveEncoder.getDistance();

  }

  public double getRightWheelCm() {
    return rightWheelEncoder.getDistance();

  }

  public double getLeftWheelCm() {
    return leftWheelEncoder.getDistance();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
}
