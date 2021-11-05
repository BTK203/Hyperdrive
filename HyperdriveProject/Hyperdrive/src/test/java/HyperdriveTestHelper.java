import frc.robot.Constants;
import frc.robot.util.hyperdrive.emulation.PathEmulator;
import frc.robot.util.hyperdrive.enumeration.DriveStyle;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Units;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** 
 * Helper for the {@link HyperdriveTests} class.
 */
public class HyperdriveTestHelper {
    /**
     * Calculates and returns a velocity map for the given path.
     * @param path The path to calculate a velocity map for
     * @return The velocity map for the given {@code path}.
     */
    public static double[] calculateVelocityMapForPath(Path path) {
        //create a fake path emulator and calculate velocity maps on it
        PathEmulator emulator = new PathEmulator(DriveStyle.TANK, 1, Units.LENGTH.METERS, 120, Units.FORCE.POUND);
        emulator.load(path, Constants.TEST_PARAMS);
        
        //this call will populate the emulator's velocity map
        emulator.performInitialCalculations(path.getPoints()[0]);

        return emulator.getVelocityMap();
    }
}
