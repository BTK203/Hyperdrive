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

    //these vary depending on setup of motor controllers
    public static final int
        LEFT_MASTER_ID = 1,
        LEFT_SLAVE_ID = 2,
        RIGHT_MASTER_ID = 3,
        RIGHT_SLAVE_ID = 4;

    //these vary depending on setup of motor controllers
    public static final boolean
        LEFT_MASTER_INVERT  = true,
        LEFT_SLAVE_INVERT   = true,
        RIGHT_MASTER_INVERT = false,
        RIGHT_SLAVE_INVERT  = false;

    public static final int DRIVE_AMP_LIMIT = 60; //amps

    public static final double
        MOTOR_ROTATIONS_PER_INCH = 0.412, 
        ROBOT_WHEELBASE_WIDTH = 24; //inches
}
