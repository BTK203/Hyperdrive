// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.hyperdrive.emulation.ConstantEmulationParams;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.util.PIDFAConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int
        LEFT_MASTER_ID = 1,
        LEFT_SLAVE_ID = 2,
        RIGHT_MASTER_ID = 3,
        RIGHT_SLAVE_ID = 4;

    public static final double WHEEL_BASE_WIDTH = 0.5; //meters

    public static final double KEY_THROTTLE_PER_SECOND = 5;

    //test constants. DO NOT CHANGE THESE OR ELSE YOU WILL BREAK THE TESTS AND HAVE TO REDO THEM
    public static final PIDFAConfig TEST_PIDFA = new PIDFAConfig(0.25, 1, 0.01, 0.125, 1.5, 1);
    public static final IEmulateParams TEST_PARAMS = new ConstantEmulationParams(1.27, 2.54, 0.025, 0.3048, TEST_PIDFA);
}
