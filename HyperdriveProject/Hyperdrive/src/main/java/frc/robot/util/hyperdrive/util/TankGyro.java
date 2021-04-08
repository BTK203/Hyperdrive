// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

import frc.robot.util.hyperdrive.Hyperdrive;

/** 
 * A class that calculates the heading of a tank-style robot using only the drivetrain
 * encoders. This class can be used in place of a gyro if none is present on a robot,
 * and can only drift when the robot is moving (as opposed to a physical gyro drifting
 * naturally). 
 */
public class TankGyro {
    private double
        heading,
        lastLeftPosition,
        lastRightPosition;

    private final double 
        motorUnitsPerUnit,
        wheelBaseWidth;

    /**
     * Creates a new TankGyro.
     * @param wheelBaseWidth The length between the left and right wheels of the robot, 
     * measured parallel to the width of the robot. This measurement MUST be taken
     * in the same common distance units (inches, meters, etc) that are used in the next
     * parameter, {@code motorUnitsPerUnit}. For example, if that value is in motor units per
     * inch, then this value must be in inches.
     * @param motorUnitsPerUnit The number of motor units (encoder counts, revolutions, etc)
     * that happen while the robot travels in a common distance unit (inches, meters, etc).
     * The "common distance unit" of this value MUST match the unit of {@code wheelBaseWidth}
     * @param leftPosition The current position of the left motors in motor units.
     * @param rightPosition The current position of the right motors in motor units.
     */
    public TankGyro(final double wheelBaseWidth, final double motorUnitsPerUnit, double leftPosition, double rightPosition) {
        this.wheelBaseWidth = wheelBaseWidth;
        this.motorUnitsPerUnit = motorUnitsPerUnit;

        this.heading = 0;
        this.lastLeftPosition = leftPosition / motorUnitsPerUnit;
        this.lastRightPosition = rightPosition / motorUnitsPerUnit;
    }

    /**
     * Creates a new TankGyro, initializing the left and right positions to be 0.
     * @param wheelBaseWidth The length between the left and right wheels of the robot, 
     * measured parallel to the width of the robot. This measurement MUST be taken
     * in the same common distance units (inches, meters, etc) that are used in the next
     * parameter, {@code motorUnitsPerUnit}. For example, if that value is in motor units per
     * inch, then this value must be in inches.
     * @param motorUnitsPerUnit The number of motor units (encoder counts, revolutions, etc)
     * that happen while the robot travels in a common distance unit (inches, meters, etc).
     * The "common distance unit" of this value MUST match the unit of {@code wheelBaseWidth}
     */
    public TankGyro(final double wheelBaseWidth, final double motorUnitsPerUnit) {
        this(wheelBaseWidth, motorUnitsPerUnit, 0, 0);
    }

    /**
     * Returns the heading of the robot, in degrees, as calculated using the drivetrain motor 
     * positions. This method's return value will only change when {@link #update(double, double)} 
     * is called.
     * @return The current heading of the robot in degrees.
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Updates the heading of the robot based on the positions of the drivetrain motors. 
     * This method should be called in one of the robot's {@code periodic()} methods. If 
     * this object is being used with the rest of Hyperdrive, then this method would be
     * well-placed before the {@link Hyperdrive #update(double, double, double)} method. 
     * The return value of {@link #getHeading()} will only change when this method is called.
     * @param leftPosition The position of the drivetrain's left wheels in motor units.
     * @param rightPosition The position of the drivetrain's right wheels in motor units.
     */
    public void update(double leftPosition, double rightPosition) {
        //use the formula: heading = (left distance - right distance) / wheel base width 
        double
            leftPositionUnits = leftPosition / motorUnitsPerUnit,
            rightPositionUnits = rightPosition / motorUnitsPerUnit,
            leftDisplacement = leftPositionUnits - lastLeftPosition,
            rightDisplacement = rightPositionUnits - lastRightPosition;
            
        double 
            changeInHeading = (leftDisplacement - rightDisplacement) / wheelBaseWidth, //unit: radians
            changeInHeadingDegrees = Math.toDegrees(changeInHeading);
            
        heading += changeInHeadingDegrees;

        lastLeftPosition = leftPositionUnits;
        lastRightPosition = rightPositionUnits;
    }
}
