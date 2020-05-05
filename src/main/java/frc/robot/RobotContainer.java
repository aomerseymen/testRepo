/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutonomousCommand;
import frc.robot.commands.autonomous.LeftAuto;
import frc.robot.commands.autonomous.MiddleAuto;
import frc.robot.commands.autonomous.RightAuto;
import frc.robot.commands.RunHopper;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final LiftSubsystem m_lift = new LiftSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public Joystick m_driverController = new Joystick(JoystickConstants.driverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drive.setDefaultCommand(
        new JoystickDrive(m_drive, () -> -m_driverController.getRawAxis(1), () -> m_driverController.getRawAxis(0)));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, 0).whileHeld(new RunIntake(m_intake, 0.8));

    new JoystickButton(m_driverController, 1).whileHeld(new RunIntake(m_intake, -0.8));

    new JoystickButton(m_driverController, 2).whileHeld(new RunShooter(m_shooter, 0.8));

    new JoystickButton(m_driverController, 3).whileHeld(new RunShooter(m_shooter, 0.8));

    new JoystickButton(m_driverController, 4).whenPressed(new ClimbOpen(m_climb));

    new JoystickButton(m_driverController, 5).whenPressed(new ClimbClose(m_climb));

    new JoystickButton(m_driverController, 6).toggleWhenPressed(new CompressorToggle(m_climb));

    new JoystickButton(m_driverController, 7).toggleWhenPressed(new ToggleIntake(m_intake));

    new JoystickButton(m_driverController, 8).whileHeld(new RunLift(m_lift, 0.8));

    new JoystickButton(m_driverController, 9).whileHeld(new RunLift(m_lift, -0.8));

    new JoystickButton(m_driverController, 10).whileHeld(new MoveArm(m_arm, 0.8));

    new JoystickButton(m_driverController, 11).whileHeld(new MoveArm(m_arm, -0.8));

    new JoystickButton(m_driverController, 12).whileHeld(new RunHopper(m_hopper, 0.8));

    new JoystickButton(m_driverController, 13).whileHeld(new RunHopper(m_hopper, -0.8));

    new POVButton(m_driverController, 0).whileHeld(new FieldOrientedTurnPID(m_drive, 0));
    new POVButton(m_driverController, 90).whileHeld(new FieldOrientedTurnPID(m_drive, 90));
    new POVButton(m_driverController, 180).whileHeld(new FieldOrientedTurnPID(m_drive, 180));
    new POVButton(m_driverController, 270).whileHeld(new FieldOrientedTurnPID(m_drive, 270));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Integer auto) {
    // An ExampleCommand will run in autonomous
    switch (auto) {
    case 1:
      return new MiddleAuto();
    case 2:
      return new LeftAuto();
    case 3:
      return new RightAuto();
    default:
      return null;

    }
  }
}
