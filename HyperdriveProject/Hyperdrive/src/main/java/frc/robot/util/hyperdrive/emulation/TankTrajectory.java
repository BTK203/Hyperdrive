// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Units;

/**
 * Represents the trajectory of a tank-style robot as it moves through a Path.
 * This class provides programmers with the target velocities of the left
 * and right motors. This class can be created using the {@link #TankTrajectory(Trajectory, double)} 
 * constructor, and can also be created by the {@link Trajectory} class using 
 * {@link Trajectory #getTankTrajectory()}.
 */
public class TankTrajectory extends Trajectory {
    private double
        rawLeftVelocity,
        rawRightVelocity,
        leftVelocity,
        rightVelocity;

    /**
     * Creates a new {@code TankTrajectory} using the data found in the given Trajectory.
     * @param trajectory The {@link Trajectory} to use to calculate the wheel velocities.
     * @param wheelBaseWidth The distance between the left and right wheels, measured
     * parallel to the width of the robot.
     */
    public TankTrajectory(Trajectory trajectory, double wheelBaseWidth) {
        super(trajectory);

        //calculate left and right wheel velocities.
        boolean isForwards = trajectory.getVelocity() >= 0;
        if(trajectory.getTurn() != 0) {
            double
                radius               = trajectory.getDistance() / trajectory.getTurn(),
                midWheelDisplacement = wheelBaseWidth / 2,
                leftDisplacement     = 0,
                rightDisplacement    = 0;

            //calculate the displacment of the wheels over the course of the trajectory.
            if(isForwards) {
                leftDisplacement  = trajectory.getTurn() * (radius - midWheelDisplacement);
                rightDisplacement = trajectory.getTurn() * (radius + midWheelDisplacement);
            } else {
                leftDisplacement  = trajectory.getTurn() * (radius + midWheelDisplacement);
                rightDisplacement = trajectory.getTurn() * (radius - midWheelDisplacement);
            }

            //convert those displacments to velocities using the base velocity of the robot.
            double timeInterval = trajectory.getDistance() / trajectory.getVelocity();

            double  
                unboundedLeftVelocity  = leftDisplacement / timeInterval, //currently in distance units per second
                unboundedRightVelocity = rightDisplacement / timeInterval;

            //ensure that left and right velocity are within the maximum and minimum speed of the robot to ensure that turns are as true as possible.
            boolean
                leftNegative = unboundedLeftVelocity < 0,
                rightNegative = unboundedRightVelocity < 0;

            double  
                leftSpeed = Math.abs(unboundedLeftVelocity),
                rightSpeed = Math.abs(unboundedRightVelocity);

            if(leftSpeed > parameters.getMaximumSpeed()) {
                double overflow = leftSpeed - parameters.getMaximumSpeed();
                rightSpeed -= overflow;
                leftSpeed = parameters.getMaximumSpeed();
            }

            if(rightSpeed > parameters.getMaximumSpeed()) {
                double overflow = rightSpeed - parameters.getMaximumSpeed();
                leftSpeed -= overflow;
                rightSpeed = parameters.getMaximumSpeed();
            }

            this.leftVelocity = leftNegative ? -1 * leftSpeed : leftSpeed;
            this.rightVelocity = rightNegative ? -1 * rightSpeed : rightSpeed;
        } else {
            double velocity = getVelocity();

            this.leftVelocity  = velocity;
            this.rightVelocity = velocity;
        }

        //order in which the raw velocities are set (before / after final velocities does not matter. 
        //the raw velocities only serve to preserve the original velocities before time conversions.
        this.rawLeftVelocity = leftVelocity;
        this.rawRightVelocity = rightVelocity;
    }
    
    /**
     * Converts the velocities to a specified unit of time. The velocities are originally in units per SECOND.
     * @param desired The time unit to convert to.
     * @return This Trajectory, after the convert is complete.
     */
    public TankTrajectory convertTime(Units.TIME desired) {
        double conversionFactor = HyperdriveUtil.convertTime(1, Units.TIME.SECONDS, desired); //units
    
        //get the reciprocal of the conversion factor and use that to convert the units, because velocities have time on the bottom of the fraction
        double reciprocal = 1 / conversionFactor;
        leftVelocity *= reciprocal;
        rightVelocity *= reciprocal;
        return this;
    }

    /**
     * Inverts the turn that the robot takes during the Path. Call this method if the robot turns the wrong 
     * directions while driving.
     * @return This Trajectory, after the invert is complete.
     */
    public TankTrajectory invertTurn() {
        double placeholder = rawLeftVelocity;
        rawLeftVelocity = rawRightVelocity;
        rawRightVelocity = placeholder;

        placeholder = leftVelocity;
        leftVelocity = rightVelocity;
        rightVelocity = placeholder;

        return this;
    }

    /**
     * Inverts the direction of the robot as it drives through a path. Call this method if the robot is 
     * driving backwards when it should be driving forwards, or vice versa.
     * @return This Trajectory, after the invert is complete.
     */
    public TankTrajectory invertDirection() {
        rawLeftVelocity *= -1;
        rawRightVelocity *= -1;
        leftVelocity *= -1;
        rightVelocity *= -1;

        return this;
    }

    /**
     * Returns the value of the left velocity in its original distance units per second.
     * @return Target left velocity in distance units per second.
     */
    public double getRawLeftVelocity() {
        return rawLeftVelocity;
    }

    /**
     * Returns the value of the right velocity in its original distance units per second.
     * @return Target right velocity in distance units per second.
     */
    public double getRawRightVelocity() {
        return rawRightVelocity;
    }

    /**
     * Returns the target velocity of the left drive wheels needed to stay on course 
     * with the path currently being driven. This value is returned in the raw motor controller units
     * (assuming that a call to {@link #convertTime(frc.robot.util.hyperdrive.util.Units.TIME)} was made
     * with the right time unit), so the target velocity of the motors can be directly set to this value. 
     * @return The target velocity of the left wheels.
     */
    public double getLeftVelocity() {
        return leftVelocity * getMotorUnitsPerUnit();
    }

    /**
     * Returns the target velocity of the right drive wheels in order to stay on course
     * with the path currently being driven.This value is returned in the raw motor controller units
     * (assuming that a call to {@link #convertTime(frc.robot.util.hyperdrive.util.Units.TIME)} was made
     * with the right time unit), so the target velocity of the motors can be directly set to this value. 
     * @return The target velocity of the right wheels.
     */
    public double getRightVelocity() {
        return rightVelocity * getMotorUnitsPerUnit();
    }

    /**
     * Calculates percent output of the left wheels needed to continue driving the path at this point.
     * @param currentLeftVelocity The velocity of the right motors at this moment, in motor units.
     * @return Target left percent output of the left motors.
     */
    public double getLeftPercentOutput(double currentLeftVelocity) {
        return controller.calculateOutput(TankController.LEFT_CONTROLLER, getLeftVelocity(), currentLeftVelocity);
    }

    /**
     * Calculates the percent output of the right wheels needed to continue driving the path at this point.
     * @param currentRightVelocity The velocity of the right motors at this moment, in motor units.
     * @return Target right percent output of the right motors.
     */
    public double getRightPercentOutput(double currentRightVelocity) {
        return controller.calculateOutput(TankController.RIGHT_CONTROLLER, getRightVelocity(), currentRightVelocity);
    }
}
