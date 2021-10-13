// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.PIDFAConfig;
import frc.robot.util.hyperdrive.util.Units;

/** 
 * A class that uses the Preferences table on the dashboard to determine
 * a path's emulation parameters. Use this class to tune any paths that 
 * may need to be tuned, and then replace with a {@link ConstantEmulationParams}
 * that contains the tuned values from the Preferences table.
 */
public class PreferenceEmulationParams implements IEmulateParams {
    private Units.LENGTH units;

    /**
     * Creates a new PreferenceEmulationParams.
     * @param units The units of length to use. This should be the same value that you defined Hyperdrive with.
     */
    public PreferenceEmulationParams(Units.LENGTH units) {
        this.units = units;
    }

    @Override
    public double getOverturn() {
        return HyperdriveUtil.getAndSetDouble("Emulate Overturn", HyperdriveConstants.DEFAULT_OVERTURN);
    }

    @Override
    public double getMinimumSpeed() {
        return HyperdriveUtil.getAndSetDouble("Emulate Minimum Speed", HyperdriveUtil.convertDistance(HyperdriveConstants.DEFAULT_MIN_SPEED_IPS, Units.LENGTH.INCHES, units));
    }

    @Override
    public double getMaximumSpeed() {
        return HyperdriveUtil.getAndSetDouble("Emulate Maximum Speed", HyperdriveUtil.convertDistance(HyperdriveConstants.DEFAULT_MAX_SPEED_IPS, Units.LENGTH.INCHES, units));
    }

    @Override
    public double getPositionalCorrectionInhibitor() {
        return HyperdriveUtil.getAndSetDouble("Emulate Positional Correction Inhibitor", HyperdriveConstants.DEFAULT_POSITIONAL_CORRECT_INHIBITOR);
    }

    @Override
    public double getPositionalCorrectionDistance() {
        return HyperdriveUtil.getAndSetDouble("Emulate Positional Correction Distance", HyperdriveUtil.convertDistance(HyperdriveConstants.DEFAULT_POSITIONAL_CORRECT_DISTANCE, Units.LENGTH.INCHES, units));
    }

    @Override
    public double getCoefficientOfStaticFriction() {
        return HyperdriveUtil.getAndSetDouble("Emulate Coefficient of Static Friction", HyperdriveConstants.DEFAULT_COEFFICIENT_OF_STATIC_FRICTION);
    }

    @Override
    public int getPointSkipCount() {
        return (int) HyperdriveUtil.getAndSetDouble("Emulate Points to Skip", HyperdriveConstants.DEFAULT_POINT_SKIP_COUNT);
    }

    @Override
    public int getImmediatePathSize() {
        return (int) HyperdriveUtil.getAndSetDouble("Emulate Path Size", HyperdriveConstants.DEFAULT_IMMEDIATE_PATH_SIZE);
    }
    
    @Override
    public PIDFAConfig getPIDFAConfig() {
        return new PIDFAConfig(
            HyperdriveUtil.getAndSetDouble("Hyperdrive kP", 0),
            HyperdriveUtil.getAndSetDouble("Hyperdrive kI", 0), 
            HyperdriveUtil.getAndSetDouble("Hyperdrive kD", 0), 
            HyperdriveUtil.getAndSetDouble("Hyperdrive kF", 0), 
            HyperdriveUtil.getAndSetDouble("Hyperdrive Full Acceleration", 5),
            HyperdriveUtil.getAndSetDouble("Hyperdrive Min Out", -0.4),
            HyperdriveUtil.getAndSetDouble("Hyperdrive Max Out", 0.4)
        );
    }
}
