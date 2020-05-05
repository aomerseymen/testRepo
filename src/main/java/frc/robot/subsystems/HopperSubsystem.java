/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperSubsystem.
   */
  private final Talon frontRightMotor = new Talon(Constants.HopperConstants.frontRightMotor);
  private final Talon rearLeftMotor = new Talon(Constants.HopperConstants.rearLeftMotor);

  public HopperSubsystem() {



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void RunHopper(double speed){
    frontRightMotor.set(speed);
    rearLeftMotor.set(-speed);
  }
  public void StopHopper(){
    frontRightMotor.set(0);
    rearLeftMotor.set(0);
  }
}
