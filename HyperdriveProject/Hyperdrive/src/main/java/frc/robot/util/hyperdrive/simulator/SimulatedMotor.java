// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.simulator;

/** Represents a simulated motor */
public class SimulatedMotor {
    //pre-defined motors
    public static final SimulatedMotor //values for these objects obtained from motor datasheets
        NEO       = new SimulatedMotor(-0.00045, 2.7),
        FALCON500 = new SimulatedMotor(-0.00073511, 4.69);

    //configuration values
    private double 
        torqueCurveSlope,
        initialTorque;

    /**
     * Creates a new SimulatedMotor.
     * @param torqueCurveSlope The slope of the empirical torque vs. RPM line. Should be in Nm / RPM. See motor datasheets.
     * @param initialSlope The y-intercept of the empirical torque vs. RPM line. Should be in Nm. See motor datasheets.
     */
    public SimulatedMotor(double torqueCurveSlope, double initialSlope) {
        this.torqueCurveSlope = torqueCurveSlope;
        this.initialTorque = initialSlope;
    }

    /**
     * Returns the slope of the empirical torque vs. RPM line.
     * @return Torque-per-RPM value in Nm/RPM.
     */
    public double getTorqueCurveSlope() {
        return torqueCurveSlope;
    }

    /**
     * Returns the y-intercept of the torque vs. RPM line.
     * @return Initial torque value in Nm.
     */
    public double getInitialTorque() {
        return initialTorque;
    }
}
