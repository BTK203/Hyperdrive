// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.util.PIDFAConfig;

/**
 * Parameters for the robot to use while driving through a Path. These parameters include
 * overturn, maximum and minimum speeds, the positional correction inhibitor, the positional
 * correction threshold, the Coefficient of Static Friction, the point skip count, and the 
 * immediate path size.
 */
public interface IEmulateParams {
    /**
     * The minimum speed, in units PER SECOND that the robot can be going at any given time 
     * during the path emulation. No matter the tightness, the robot will always take any 
     * turn with at least this speed. This value is measured in units per second.
     * @return Minimum speed of the robot during emulation.
     */
    public double getMinimumSpeed();

    /**
     * The maximum speed, in units PER SECOND that the robot can be going at any given time during 
     * the path emulation, and the speed that the robot will try to achieve during any straightaways. 
     * This value must be greater than the minimum speed, and should be less than the robot's actual 
     * maximum speed.
     * @return Maximum speed of the robot during emulation.
     */
    public double getMaximumSpeed();

    /**
     * Returns the inhibitor for the robot's positional correction. The greater this value, the more
     * the robot will try to remain exactly on the path that it is driving. Setting this value too high
     * might result in the robot driving in a "wavy" pattern (turning left, then right, then left again,
     * then right again, and so on). Setting this value too low might cause the robot to not correct itself
     * enough when it veers off course. This value defaults to 0.025.
     * @return Positional correction inhbitor.
     */
    public double getPositionalCorrectionInhibitor();

    /**
     * Returns the approximate coefficient of static friction between the robot's wheels, and the surface that it
     * is driving on. This value can also be used to tune the speed that the robot takes its turns. The lower 
     * this value is, the slower it takes turns. The higher this value is, the faster it takes turns.
     * This value defaults to 0.6.
     * @return Coefficient of static friction between the robot's wheels and the floor's surface.
     */
    public double getCoefficientOfStaticFriction();

    /**
     * Gets the robots PIDFA configuration, which contains the user-configurable kP, kI, kD, kF and acceleration values
     * necessary to figure out what percent output to drive the motors at. If this method is being called from a
     * {@link PreferenceEmulationParams}, all of these values will be pulled from the Preferences table on the dashboard.
     * If this method is being called from a {@link ConstantEmulationParams}, these values will be whatever they were 
     * initialized as.
     * @return PIDFA config.
     */
    public PIDFAConfig getPIDFAConfig();
}
