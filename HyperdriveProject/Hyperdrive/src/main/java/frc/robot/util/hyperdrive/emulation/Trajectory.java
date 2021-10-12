// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.Hyperdrive;

/** 
 * Represents the desired path of movement of the robot at any point while driving
 * through a path. This class is the main class, which holds only the crude turn and 
 * speed values, and cannot be used to set the target velocity of any drivetrain motors 
 * without further calculations. Users should use this class to calculate target velocities
 * or positions of their motors by using the {@link #getTankTrajectory(double wheelBaseWidth)} 
 * method. More similar methods will be implemented in the future if support for different 
 * drivetrains such as swerve drive is added.
 */
public class Trajectory {
    protected double
        velocity,
        distance,
        turn;

    protected final IEmulateParams parameters;
    protected final IController controller;
    protected final double motorUnitsPerUnit;

    /**
     * Creates a new Trajectory, defining that the robot should drive at the given velocity for 
     * a certain distance ahead, while turning to the angular displacement given by turnAhead.
     * @param velocity The velocity of the robot.
     * @param distance The length of the path immediately ahead of the robot.
     * @param turn The magnitude of the turn immediately ahead of the robot, in radians.
     * @param parameters The parameters being used for driving.
     * @param controller The controller to use to calculate outputs
     * @param motorUnitsPerUnit A scalar to convert from motor units to actual units. See the 
     * {@link Hyperdrive} constructor for more.
     */
    public Trajectory(
        double velocity, 
        double distance,
        double turn, 
        final IEmulateParams parameters, 
        final IController controller, 
        final double motorUnitsPerUnit
    ) {
        this.velocity   = velocity;
        this.distance   = distance;
        this.turn       = turn;
        this.parameters = parameters;
        this.controller = controller;
        this.motorUnitsPerUnit = motorUnitsPerUnit;
    }

    /**
     * Creates a new Trajectory, copying the specified Trajectory
     * @param trajectory The trajectory to copy.
     */
    public Trajectory(Trajectory trajectory) {
        this(
            trajectory.getVelocity(), 
            trajectory.getDistance(), 
            trajectory.getTurn(), 
            trajectory.getParameters(),
            trajectory.getController(),
            trajectory.getMotorUnitsPerUnit()
        );
    }

    /**
     * Returns the target velocity of the robot as it goes through this trajectory.
     * @return Velocity of the robot.
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Returns the distance that the robot will cover while moving through the trajectory.
     * @return Distance of the trajectory.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Returns the angular displacment, in radians, of the robot as it moves through the trajectory.
     * @return The turn of the robot.
     */
    public double getTurn() {
        return turn;
    }

    /**
     * Returns the parameters that this Trajectory uses.
     * @return Parameters used to drive the robot.
     */
    public IEmulateParams getParameters() {
        return parameters;
    }

    /**
     * Returns the controller that this Trajectory uses.
     * @return Controller used to drive the robot.
     */
    public IController getController() {
        return controller;
    }

    /**
     * Returns the trajectory information in a format that can drive tank-style robots.
     * @param wheelBaseWidth The distance between the left and right wheels, measured parallel to the width of the robot.
     * @return A TankTrajectory, which contains the left and right wheel velocities required to keep the robot on course.
     */
    public TankTrajectory getTankTrajectory(double wheelBaseWidth) {
        return new TankTrajectory(this, wheelBaseWidth);
    }

    /**
     * Returns the units per unit value that was passed into the constructor.
     * @return Motor units per unit scalar.
     */
    public final double getMotorUnitsPerUnit() {
        return motorUnitsPerUnit;
    }
}
