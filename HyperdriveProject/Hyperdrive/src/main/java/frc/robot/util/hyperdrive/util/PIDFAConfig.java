// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

/** 
 * Represents a PIDF configuration with maximum acceleration
 */
public class PIDFAConfig {
    private double
        kP,
        kI,
        kD,
        kF,
        accel;

    /**
     * Creates a new PIDFAConfig.
     * @param kP The kP value to use.
     * @param kI The kI value to use.
     * @param kD The kD value to use.
     * @param kF The kF value to use.
     * @param accel The maximum acceleration of the robot at full throttle.
     */
    public PIDFAConfig(double kP, double kI, double kD, double kF, double accel) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.accel = accel;
    }

    /**
     * Returns the kP (proportional gain) value of this PIDFAConfig.
     * Robot will be driven such that {@code percentOut = P + I + D + F},
     * where {@code P = error * kP}
     * 
     * @return kP value.
     */
    public double getkP() {
        return kP;
    }

    /**
     * Returns the kI value of this PIDFAConfig.
     * Robot will be driven such that {@code percentOut = P + I + D + F},
     * where {@code I = sum of all past error * kI}
     * @return kI value.
     */
    public double getkI() {
        return kI;
    }

    /**
     * Returns the kD value of this PIDFAConfig.
     * Robot will be driven such that {@code percentOut = P + I + D + F},
     * where {@code D = rate of change of error * kD}
     * @return kD value.
     */
    public double getkD() {
        return kD;
    }

    /**
     * Returns the kF value of this PIDFAConfig.
     * Robot will be driven such that {@code percentOut = P + I + D + F},
     * where {@code F = kF}
     * @return kF value.
     */
    public double getkF() {
        return kF;
    }

    /**
     * Returns the maximum acceleration of the robot at full throttle.
     * @return Acceleration at full throttle.
     */
    public double getAccel() {
        return accel;
    }
}
