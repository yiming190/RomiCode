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
    public static final class DrivetrainConstants {
        public static final int LEFT_MOTOR_PORT = 0;
        public static final int RIGHT_MOTOR_PORT = 1;
        public static final int LEFT_ENCODER_A = 4;
        public static final int LEFT_ENCODER_B = 5;
        public static final int RIGHT_ENCODER_A = 6;
        public static final int RIGHT_ENCODER_B = 7;

        public static final double COUNTS_PER_REVOLUTION = 1440.0;
        public static final double WHEEL_DIAMETER_INCH = 2.75591;
        public static final double INCHES_PER_PULSE = (Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION;
    }
}
