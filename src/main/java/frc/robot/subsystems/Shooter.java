/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final VictorSP rightShooterMotor = new VictorSP(ShooterConstants.rightShooterMotorPort);
  private final VictorSP leftShooterMotor = new VictorSP(ShooterConstants.leftShooterMotorPort);

  
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void runShooter(double speed){
    rightShooterMotor.set(speed);
    leftShooterMotor.set(speed);
  }
  public void stopShooter()
  {
    rightShooterMotor.set(0);
    leftShooterMotor.set(0);
  }
}
