/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.RunLift;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousCommand.
   */
  public AutonomousCommand(LiftSubsystem m_lift, ArmSubsystem m_arm, Shooter m_shooter, DriveSubsystem m_drive,
      HopperSubsystem m_hopper, Intake m_intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // alongwith- birlikte çalıştırmak için // withtimeout-- girdigin sure kadar
    // yapar
    // (new RunShooter(m_shooter, 1).withTimeout(0.75),new RunLift(m_lift,
    // 0.5).raceWith(new RunHopper(m_hopper, 0.8)),
    // new AutonomousDrive(m_drive, 0.8, 300).withTimeout(3));

    super(new RunShooter(m_shooter, 0.8).withTimeout(0.75),
        new RunShooter(m_shooter, 0.8).raceWith(new RunHopper(m_hopper, 0.8)).withTimeout(2.5),
        new AutonomousDrive(m_drive, -0.8, -300).raceWith(new RunIntake(m_intake, 0.8)
            .raceWith(new RunHopper(m_hopper, 0.8).raceWith(new RunShooter(m_shooter, 0.3),
                new AutonomousDrive(m_drive, 0.8, 300), new RunShooter(m_shooter, 0.8).withTimeout(0.75),
                new RunShooter(m_shooter, 0.8).raceWith(new RunHopper(m_hopper, 0.8).withTimeout(2.5))))));

  }
}
