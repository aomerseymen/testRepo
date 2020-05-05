/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
    public final class IntakeConstants {
        public static final int rightIntakeMotorPort = 0;
        public static final int leftIntakeMotorPort = 1;
        public static final int solenoidForwardPin = 0;
        public static final int solenoidBackwardPin = 1;

    }

    public final class JoystickConstants {
        public static final int driverControllerPort = 0;

    }

    public final class ShooterConstants {
        public static final int rightShooterMotorPort = 2;
        public static final int leftShooterMotorPort = 3;

    }

    public static class DriveConstants {
        public static int frontLeftMotorPin = 4;
        public static int frontRightMotorPin = 5;
        public static int rearLeftMotorPin = 6;
        public static int rearRightMotorPin = 7;
        public static int driveEncoder_A = 0;
        public static int driveEncoder_B = 1;
        public static double wheelPerimeter = 6 * 2.54 * Math.PI;

        public static final double turnP = 1.0;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;

        public static final double driveP = 1.0;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;

        public static final double turnAccuracy = 2.0;
        public static final double driveAccuracy = 3.0;

        public static int rightWheelEncoder_A = 6;
        public static int rightWheelEncoder_B = 7;
        public static int leftWheelEncoder_A = 8;
        public static int leftWheelEncoder_B = 9;

    }

    public final class ClimbConstants {
        public static final int solenoidForwardPin = 2;
        public static final int solenoidBackwardPin = 3;
        public static final int compressorPin = 0;

    }

    public final class LiftConstants {
        public static final int rightLiftMotor = 8;
        public static final int leftLiftMotor = 9;
        public static final int topLimitSwitchPort = 6;
        public static final int bottomLimitSwitchPort = 7;
        public static final int liftEncoder_A = 2;
        public static final int liftEncoder_B = 3;

    }

    public final class ArmConstants {
        public static final int armMotorPort = 10;
        public static final int armPotPort = 0;
        public static final int armEncoder_A = 4;
        public static final int armEncoder_B = 5;

    }

    public final class HopperConstants {
        public static final int frontRightMotor = 12;
        public static final int rearLeftMotor = 13;
    }

}
