// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.HyperdriveConstants;

/** 
 * A class that tracks the position of the robot in field space.
 * Because team 3695 has not had experience with swerve drive as of yet,
 * this class is primarly programmed for tank bots.
 * <br><br>
 * This class relies on the integrity of the drivetrain encoder readouts to calculate
 * the robot's position on the field. Those encoder counts must be as close as
 * possible to the actual displacement of the wheel from its starting position. That
 * being said, in order for this class to accurately report the robot's true position
 * on the field, the drivetrain wheels cannot drift. Drifting will result in a difference 
 * betweenthe encoder position and the drivetrain's actual position, which will cause this 
 * class to inaccurately report the robot's position.
 */
public class PositionTracker {
    private double
        lastDistance,
        x,
        y,
        heading;

    private boolean waitForEncoders; //indicates if the PositionTracker is to wait for the distance readout to zero before zeroing position.
    
    private final double motorUnitsPerUnit;

    /**
     * Creates a new PositionTracker.
     * @param motorUnitsPerUnit The number of "motor units" that are present in a single unit of distance (inches,
     * meters, etc). This should be the same value that was passed into the {@link Hyperdrive} constructor.
     * @param x The starting X-coordinate of the robot.
     * @param y The starting Y-coordinate of the robot.
     * @param heading The starting heading angle of the robot in degrees. (0 = towards positive X. Positive = CCW)
     */
    public PositionTracker(final double motorUnitsPerUnit, double x, double y, double heading) {
        this.motorUnitsPerUnit = motorUnitsPerUnit;
        waitForEncoders = false;
        setPositionAndHeading(x, y, heading);
    }

    /**
     * Creates a new Position tracker, with starting position and rotation at 0.
     * @param motorUnitsPerUnit The number of "motor units" that are present in a single unit of distance (inches,
     * meters, etc). This should be the same value that was passed into the {@link Hyperdrive} constructor.
     */
    public PositionTracker(final double motorUnitsPerUnit) {
        this(motorUnitsPerUnit, 0, 0, 0);
    }

    /**
     * Sets the position and heading of the robot.
     * @param x The new X-coordinate of the robot.
     * @param y The new Y-coordinate of the robot.
     * @param heading The new heading angle of the robot.
     */
    public void setPositionAndHeading(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Sets the position and heading of the robot.
     * @param newPositionAndHeading The new position and heading of the robot.
     */
    public void setPositionAndHeading(Point2D newPositionAndHeading) {
        setPositionAndHeading(newPositionAndHeading.getX(), newPositionAndHeading.getY(), newPositionAndHeading.getHeading());
    }

    /**
     * Zeroes the position and heading of the robot. This should be called when the 
     * robot is NOT moving.
     * @param waitForEncoders True if the drivetrain is also currently being zeroed.
     * This option is necessary because of the lag that is experienced when performing a 
     * CAN operation. An encoder zero operation will not instantly result in zero encoders.
     * In fact, the encoder value will most likely not read 0 until the next robot frame.
     * Enabling this option forces this method to wait until the either class is fed  
     * a position value that is close enough to zero, or until a half-second has passed.
     */
    public void zeroPositionAndHeading(boolean waitForEncoders) {
        this.waitForEncoders = waitForEncoders;

        if(!waitForEncoders) {
            setPositionAndHeading(0, 0, 0);
        }
    }

    /**
     * Zeroes the position and heading of the robot. This is equivelent to calling
     * {@code zeroPositonAndHeading(false)}.
     */
    public void zeroPositionAndHeading() {
        zeroPositionAndHeading(false);
    }

    /**
     * Updates the position of the robot using a distance travelled and heading travelled in.
     * If the {@link Hyperdrive} class is present, this method should not be called. Call
     * one of Hyperdrive's {@code update()} methods, and this will be called by those. 
     * <br><br>
     * 
     * It is recommended that this method is called in one of the robot's {@code periodic()} 
     * methods. While is is possible to call this method in a while loop in some separate thread,
     * programmers should be careful of how often they request encoder counts from their motor controllers
     * to avoid overloading their CAN network, which can cause undesired motor behavior.
     * @param driveDistance The displacement of the robot since the last zero, in encoder counts.
     * @param movementDirection The direction in which the robot is moving.
     */
    public void update(double driveDistance, double movementDirection) {
        movementDirection %= 360;
        double averageHeading = (movementDirection + this.heading) / 2;
        double distanceSinceLastUpdate = driveDistance - lastDistance;

        double linearDisplacement = 0;
        double changeInHeading = Math.toRadians(HyperdriveUtil.getAngleToHeading(movementDirection, this.heading));
        if(changeInHeading != 0 && HyperdriveConstants.USE_CHORD_BASED_TRACKING) {
            //use the arc chord-based displacement method to increase the accuracy of the tracking.
            double radiusOfArc = distanceSinceLastUpdate / changeInHeading;

            //use the chord length formula, crd = r * 2sin(theta / 2) to get the straight-line displacement.
            linearDisplacement = radiusOfArc * 2 * Math.sin(changeInHeading / 2);
        } else {
            linearDisplacement = distanceSinceLastUpdate;
        }

        lastDistance = driveDistance;

        //break vector into components
        double headingRads = Math.toRadians(averageHeading);
        double driveX = linearDisplacement * Math.cos(headingRads);
        double driveY = linearDisplacement * Math.sin(headingRads);

        //convert to distance units
        driveX /= motorUnitsPerUnit;
        driveY /= motorUnitsPerUnit;

        this.x += driveX;
        this.y += driveY;
        this.heading = movementDirection;

        if(waitForEncoders && Math.abs(driveDistance) < HyperdriveConstants.ALMOST_ZERO) {
            setPositionAndHeading(0, 0, 0);
            waitForEncoders = false;
        }
    }

    /**
     * Returns the current position and heading of the robot.
     * @return Position and Heading of the robot.
     */
    public Point2D getPositionAndHeading() {
        return new Point2D(x, y, heading);
    }
}
