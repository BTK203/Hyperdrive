// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.HyperdriveConstants;

/** 
 * A class that calculates the heading of a tank-style robot using only the drivetrain
 * encoders. In its current state of development, this class has shown to be reliable for 
 * constant velocity and slightly varied acceleration cases, but drifts as far as 20 degrees 
 * in 5 seconds when motor speed and robot direction is frequently varied. Therefore, this
 * class should not replace an actual gyro on competition robots but can be used for tests
 * and experiments.
 * <br><br>
 * This class' functionality is dependent on the positions reported by the drivetrain's 
 * motor encoders. That being said, in order for the heading reported by this class to
 * stay accurate, the wheels CANNOT DRIFT. When the robot drifts or slides, the positions
 * reported by the encoders no longer truly represent the displacement of that wheel from
 * it's starting position. Therefore, the gyro cannot accurately calculate the robots
 * angular displacement from its starting heading.
 */
public class TankGyro {
    private double
        heading, //degrees
        lastLeftPosition, //common units
        lastRightPosition, //common units
        lastLeftDisplacement, //...
        lastRightDisplacement, //...
        lastLeftVelocity, //common units per second
        lastRightVelocity; //common units per second

    private boolean 
        inverted,    
        waitForEncoders;

    private long lastUpdatedTime;

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
     * @param inverted {@code true} if the gyro is inverted, {@code false} otherwise.
     * @param leftPosition The current position of the left motors in motor units.
     * @param rightPosition The current position of the right motors in motor units.
     */
    public TankGyro(final double wheelBaseWidth, final double motorUnitsPerUnit, boolean inverted, double leftPosition, double rightPosition) {
        this.wheelBaseWidth = wheelBaseWidth;
        this.motorUnitsPerUnit = motorUnitsPerUnit;
        this.inverted = inverted;
        this.heading = 0;
        this.lastLeftPosition = leftPosition / motorUnitsPerUnit;
        this.lastRightPosition = rightPosition / motorUnitsPerUnit;
        this.waitForEncoders = false;
        this.lastLeftVelocity = 0;
        this.lastRightVelocity = 0;
        this.lastUpdatedTime = System.currentTimeMillis();
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
     * @param inverted {@code true} if the gyro is inverted, {@code false} otherwise.
     */
    public TankGyro(final double wheelBaseWidth, final double motorUnitsPerUnit, boolean inverted) {
        this(wheelBaseWidth, motorUnitsPerUnit, inverted, 0, 0);
    }

    /**
     * Creates a new TankGyro, initializing the left and right positions to be 0, and
     * {@code inverted} as {@code false}.
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
        this(wheelBaseWidth, motorUnitsPerUnit, false);
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
     * Inverts (or un-inverts) the gyro.
     * @param inverted {@code true} if the gyro should be inverted, {@code false} otherwise.
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
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
        // this method uses velocities to calculate heading instead of simply using the positions directly to
        // reduce drift that is caused by acceleration.

        long currentTime = System.currentTimeMillis();

        //calculate the current velocity of the motors
        double
            deltaTime = (currentTime - lastUpdatedTime) / 1000.0,
            leftVelocity = (lastLeftPosition - leftPosition) / deltaTime,
            rightVelocity = (lastRightPosition - rightPosition) / deltaTime,
            leftVelocityUnits = leftVelocity / motorUnitsPerUnit,
            rightVelocityUnits = rightVelocity / motorUnitsPerUnit;

        //use the change in velocities to calculate the change in position for both left and right sides.
        double 
            leftDisplacementNow = deltaTime * (0.5 * (leftVelocityUnits - lastLeftVelocity) + lastLeftVelocity),
            rightDisplacementNow = deltaTime * (0.5 * (rightVelocityUnits - lastRightVelocity) + lastRightVelocity),
            leftDisplacement = (leftDisplacementNow + lastLeftDisplacement) / 2, //reduce noise from riemann sum
            rightDisplacement = (rightDisplacementNow + lastRightDisplacement) / 2;

        double changeInHeading = 0; //unit: radians
        if(inverted) {
            changeInHeading = (leftDisplacement - rightDisplacement) / wheelBaseWidth;
        } else { //NOT inverted
            changeInHeading = (rightDisplacement - leftDisplacement) / wheelBaseWidth;
        }

        double changeInHeadingDegrees = Math.toDegrees(changeInHeading);
        if(!Double.isNaN(changeInHeadingDegrees)) { //sometimes happens when the robot initializes really fast (deltaTime is 0)
            heading += changeInHeadingDegrees;
        }

        lastLeftPosition      = leftPosition;
        lastRightPosition     = rightPosition;
        lastLeftVelocity      = leftVelocityUnits;
        lastRightVelocity     = rightVelocityUnits;
        lastLeftDisplacement  = leftDisplacementNow;
        lastRightDisplacement = rightDisplacementNow;
        lastUpdatedTime       = currentTime;

        if(waitForEncoders && drivetrainAtZero(leftPosition, rightPosition)) {
            setHeading(0);
            lastLeftPosition = 0;
            lastRightPosition = 0;
            lastLeftVelocity = 0;
            lastRightVelocity = 0;
            lastLeftDisplacement = 0;
            lastRightDisplacement = 0;
            waitForEncoders = false;
        }
    }

    /**
     * Sets the heading of the TankGyro.
     * @param heading The new heading of the tank gyro, in degrees.
     */
    public void setHeading(double heading) {
        this.heading = heading;
    }

    /**
     * Sets the heading of the TankGyro to 0. This method offers the option to wait for 
     * the drivetrain encoders to read out a positional value of 0 before it does zero. This
     * option should be enabled if the drivetrain motors are being zeroed along with the gyro,
     * or else the heading could be set to some unpredictable value due to the amount of time it
     * takes for a drivetrain encoder zero operation to actually complete (a good explanation of
     * how this works can be found in the javadoc for {@link PositionTracker #zeroPositionAndHeading(boolean)}).
     * @param waitForEncoders {@code true} if the TankGyro should wait for the drivetrain encoders to 
     * zero before zeroing the heading. If this option is enabled, then the heading will not be zeroed
     * until {@link #update(double, double)} is called with zero as both position values. If you
     * only wish to zero the heading of the gyro, and are not zeroing the drivetrain encoders along
     * with it, then this value should be {@code false}.
     */
    public void zeroHeading(boolean waitForEncoders) {
        this.waitForEncoders = waitForEncoders;

        if(!waitForEncoders) {
            setHeading(0);
        }
    }

    /**
     * Zeros the heading of the gyro, but does not wait for the drivetrain encoders to be zero. Calling 
     * this method is equivilent to calling {@code zeroHeading(false)}.
     */
    public void zeroHeading() {
        zeroHeading(false);
    }

    /**
     * Returns whether or not the drivetrain positions are at (or close enough to) zero.
     * @param left The position of the left motor, in motor units
     * @param right The position of the right motor, in motor units
     * @return {@code true} if the drivetrain encoders are zeroed, {@code false} otherwise.
     */
    private boolean drivetrainAtZero(double left, double right) {
        return Math.abs(left) < HyperdriveConstants.ALMOST_ZERO && Math.abs(right) < HyperdriveConstants.ALMOST_ZERO;
    }
}
