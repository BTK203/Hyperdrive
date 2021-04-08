// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

/** 
 * Describes a unit of measurement, in time and distance.
 */
public class Units {

    /**
     * Describes units of distance.
     */
    public static enum DISTANCE {
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
         * Similarly human readable, but also the common time unit in Spark MAX motor controllers.
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
