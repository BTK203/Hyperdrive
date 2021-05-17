// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Units;

/** 
 * A set of parameters that a {@code Path} will use to drive the robot
 * through it. Default values can be accessed using ConstantEmulationParams.getDefaults().
 * If you wish to tune the parameters for your robot, use the {@link PreferenceEmulationParams}
 * class instead.
 */
public class ConstantEmulationParams implements IEmulateParams {
    private double  
        overturn,
        minimumSpeed,
        maximumSpeed,
        positionalCorrectionInhibitor,
        positionalCorrectionDistance,
        coefficientOfStaticFriction;

    private int
        pointSkipCount,
        immediatePathSize;

    /**
     * Creates a new ConstantEmulationParams, describing the parameters that are to be followed as the robot emulates a {@code Path}.
     * @param overturn Increases or decreases the magnitude of any future turn of the {@code Path} by multiplying the turn by this amount.
     * @param minimumSpeed The minimum speed of the robot at any time during the {@code Path}
     * @param maximumSpeed The maximum speed of the robot at any time during the {@code Path}
     * @param positionalCorrectionInhibitor Inhibits the robot from performing positional corrections during the {@code Path}.
     * @param positionalCorrectionDistance The minimuim distance off-course that the robot has to be to start correcting its position.
     * @param coefficientOfStaticFriction The coefficient of static friction between the robot's wheels and the floor's surface. Lower values = slower turns.
     * @param pointSkipCount The number of points ahead of the robot to ignore when making decisions about turns.
     * @param immediatePathSize The number of points ahead of the robot (after pointSkipCount) to look at to make desicions about turns.
     */
    public ConstantEmulationParams(
        double overturn,
        double minimumSpeed,
        double maximumSpeed,
        double positionalCorrectionInhibitor,
        double positionalCorrectionDistance,
        double coefficientOfStaticFriction,
        int pointSkipCount,
        int immediatePathSize
    ) {
        this.overturn = overturn;
        this.minimumSpeed = minimumSpeed;
        this.maximumSpeed = maximumSpeed;
        this.positionalCorrectionInhibitor = positionalCorrectionInhibitor;
        this.positionalCorrectionDistance = positionalCorrectionDistance;
        this.coefficientOfStaticFriction = coefficientOfStaticFriction;
        this.pointSkipCount = pointSkipCount;
        this.immediatePathSize = immediatePathSize;
    }

    @Override
    public double getOverturn() {
        return overturn;
    }

    @Override
    public double getMinimumSpeed() {
        return minimumSpeed;
    }

    @Override
    public double getMaximumSpeed() {
        return maximumSpeed;
    }

    @Override
    public double getPositionalCorrectionInhibitor() {
        return positionalCorrectionInhibitor;
    }

    @Override
    public double getPositionalCorrectionDistance() {
        return positionalCorrectionDistance;
    }

    @Override
    public double getCoefficientOfStaticFriction() {
        return coefficientOfStaticFriction;
    }

    @Override
    public int getPointSkipCount() {
        return pointSkipCount;
    }

    @Override
    public int getImmediatePathSize() {
        return immediatePathSize;
    }
    
    /**
     * Returns the default values for robots to use, in whatever unit is supplied.
     * @return A {@link ConstantEmulationParams} containing the default parameters for path driving.
     */
    public static ConstantEmulationParams getDefaults(Units.DISTANCE units) {
        return new ConstantEmulationParams(
            HyperdriveConstants.DEFAULT_OVERTURN,
            HyperdriveUtil.convertDistance(HyperdriveConstants.DEFAULT_MIN_SPEED_IPS, Units.DISTANCE.INCHES, units),
            HyperdriveUtil.convertDistance(HyperdriveConstants.DEFAULT_MAX_SPEED_IPS, Units.DISTANCE.INCHES, units),
            HyperdriveConstants.DEFAULT_POSITIONAL_CORRECT_INHIBITOR,
            HyperdriveConstants.DEFAULT_POSITIONAL_CORRECT_DISTANCE,
            HyperdriveConstants.DEFAULT_COEFFICIENT_OF_STATIC_FRICTION,
            HyperdriveConstants.DEFAULT_POINT_SKIP_COUNT,
            HyperdriveConstants.DEFAULT_IMMEDIATE_PATH_SIZE
        );
    }
}
