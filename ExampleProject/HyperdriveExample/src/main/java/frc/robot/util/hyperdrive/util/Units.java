// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

/** 
 * Describes a unit of measurement of length, time, and force.
 */
public class Units {

    /**
     * Describes units of length or distance.
     */
    public static enum LENGTH {
        INCHES,
        FEET,
        YARDS,
        CENTIMETERS,
        METERS
    }

    /**
     * Describes units of time.
     */
    public static enum TIME {
        /**
         * The common time unit for CTRE motor controllers.
         */
        DECASECONDS,

        /**
         * Regular, human-readable time.
         */
        SECONDS,

        /**
         * The common time unit in REV motor controllers.
         */
        MINUTES
    }

    /**
     * Describes units of force or weight.
     */
    public static enum FORCE {
        NEWTON,
        POUND,
        KILOGRAM_FORCE;
    }
}
