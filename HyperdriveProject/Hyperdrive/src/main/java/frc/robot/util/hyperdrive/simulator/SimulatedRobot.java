// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.simulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.PositionTracker;
import frc.robot.util.hyperdrive.util.TankGyro;
import frc.robot.util.hyperdrive.util.Units;

/** 
 * Keeps information about the robot's configuration.
 * ALL PARAMETERS FOR ALL METHODS SHOULD BE IN SI UNITS.
 */
public class SimulatedRobot  {
    //configuration values
    private SimulatedMotor motor;
    private int motorsPerSide;
    private double  
        wheelRadius,
        gearRatio,
        robotMass,
        motorUnitsPerUnit;

    //runtime values
    private double
        leftVelocity,
        rightVelocity,
        lastTimeInterval;

    private double
        leftPositionMotorUnits,
        rightPositionMotorUnits;
    
    //tools
    private TankGyro gyro;
    private PositionTracker positionTracker;

    /**
     * Creates a new SimulatedRobot. Uses the default motors per side value of 2
     * and the default motor units per unit value of 0.472.
     * @param motor The type of motor being used.
     * @param wheelRadius The radius of the drive wheels.
     * @param wheelBaseWidth The length between the two sets of wheels on the simulated robot
     * @param gearRatio The gear ratio of the gearbox (wheel RPM / motor RPM)
     * @param robotWeight The weight of the robot.
     * @param weightUnit The units used to specify the weight.
     */
    public SimulatedRobot(
        SimulatedMotor motor, 
        double wheelRadius, 
        double wheelBaseWidth,
        double gearRatio, 
        double robotWeight, 
        Units.FORCE weightUnit
    ) {
        this(motor, 2, wheelRadius, wheelBaseWidth, gearRatio, robotWeight, weightUnit, 0.472);
    }

    /**
     * Creates a new SimulatedRobot.
     * @param motor The type of motor being used.
     * @param motorsPerSide The number of motors per side.
     * @param wheelRadius The radius of the drive wheels.
     * @param wheelBaseWidth The width between the sets of wheels of the robot.
     * @param gearRatio The gear ratio of the gearbox (wheel RPM / motorRPM);
     * @param robotWeight The weight of the robot.
     * @param weightUnit The units used to specify the weight.
     * @param motorUnitsPerUnit The number of motor units (RPM, ticks, etc) per length value (inches, meters, etc)
     */
    public SimulatedRobot(
        SimulatedMotor motor, 
        int motorsPerSide,
        double wheelRadius,
        double wheelBaseWidth,
        double gearRatio, 
        double robotWeight,
        Units.FORCE weightUnit, 
        double motorUnitsPerUnit
    ) {
        this.motor = motor;
        this.motorsPerSide = motorsPerSide;
        this.wheelRadius = wheelRadius;
        this.gearRatio = gearRatio;
        this.robotMass = HyperdriveUtil.massKGFromWeight(robotWeight, weightUnit);
        this.motorUnitsPerUnit = motorUnitsPerUnit;
        leftVelocity = 0;
        rightVelocity = 0;
        leftPositionMotorUnits = 0;
        rightPositionMotorUnits = 0;
        positionTracker = new PositionTracker(motorUnitsPerUnit);
        gyro = new TankGyro(wheelBaseWidth, motorUnitsPerUnit);
    }

    /**
     * Returns the simulated robot's gyro.
     * @return The gyro.
     */
    public TankGyro getGyro() {
        return gyro;
    }

    /**
     * Returns the current position of the simulated robot.
     * @return Current position and heading of the robot.
     */
    public Point2D getCurrentPositionAndHeading() {
        return positionTracker.getPositionAndHeading(); //will have updated heading
    }

    /**
     * Sets the position and heading of the simulated robot.
     * @param positionAndHeading The new position and heading of the simulated robot.
     */
    public void setCurrentPositionAndHeading(Point2D positionAndHeading) {
        positionTracker.setPositionAndHeading(positionAndHeading);
        gyro.setHeading(positionAndHeading.getHeading());
    }

    /**
     * Zeros the position and heading of the robot. Because the robot is simulated, it 
     * may appear as though it teleports back to position (0, 0) when this method is called.
     * This is expected.
     */
    public void zeroPositionAndHeading() {
        leftPositionMotorUnits = 0;
        rightPositionMotorUnits = 0;
        setCurrentPositionAndHeading(new Point2D(0, 0, 0));
    }

    /**
     * Returns the current left velocity of the robot.
     * @return Linear velocity of robot's left wheels
     */
    public double getLeftVelocity() {
        return leftVelocity;
    }

    /**
     * Returns the current right velocity of the robot.
     * @return Linear velocity of robot's right wheels
     */
    public double getRightVelocity() {
        return rightVelocity;
    }

    /**
     * Calculates the new position of the robot if it was driven for a certain time interval.
     * @param leftPercentOutput The percent output of the left motors.
     * @param rightPercentOutput The percent output of the right motors.
     * @param timeInterval The time interval for which the robot drives.
     * @return The position of the robot after it drives for the specified time interval
     */
    public Point2D update(double leftPercentOutput, double rightPercentOutput, double timeInterval) {
        //make sure leftPercentOutput and rightPercentOutput are between -1 and 1
        leftPercentOutput = (leftPercentOutput < -1 ? -1 : (leftPercentOutput > 1 ? 1 : leftPercentOutput));
        rightPercentOutput = (rightPercentOutput < -1 ? -1 : (rightPercentOutput > 1 ? 1 : rightPercentOutput));

        boolean
            leftBraking = isBraking(leftPercentOutput, leftVelocity, lastTimeInterval),
            rightBraking = isBraking(rightPercentOutput, rightVelocity, lastTimeInterval);

        SmartDashboard.putBoolean("left braking", leftBraking);

        //calculate the linear displacements of the wheels
        double
            leftDisplacement = calculateLinearDisplacement(leftPercentOutput, leftVelocity, timeInterval, leftBraking),
            rightDisplacement = calculateLinearDisplacement(rightPercentOutput, rightVelocity, timeInterval, rightBraking);

        SmartDashboard.putNumber("leftPercent", leftPercentOutput);
        SmartDashboard.putNumber("leftVelocity", leftVelocity);
        SmartDashboard.putNumber("leftDisplacement", leftDisplacement);
        SmartDashboard.putNumber("timeInterval", timeInterval);

        //update velocities
        leftVelocity = calculateNewVelocity(leftPercentOutput, leftVelocity, timeInterval, leftBraking);
        rightVelocity = calculateNewVelocity(rightPercentOutput, rightVelocity, timeInterval, rightBraking);

        SmartDashboard.putNumber("left velocity after", leftVelocity);

        leftVelocity = (Math.abs(leftVelocity) < 0.01 ? 0 : leftVelocity);
        rightVelocity = (Math.abs(rightVelocity) < 0.01 ? 0 : rightVelocity);

        //update time interval
        lastTimeInterval = timeInterval;

        //update motor positions
        leftPositionMotorUnits += leftDisplacement * motorUnitsPerUnit;
        rightPositionMotorUnits += rightDisplacement * motorUnitsPerUnit;
        double distanceDrivenMotorUnits = (leftPositionMotorUnits + rightPositionMotorUnits) / 2.0;

        SmartDashboard.putNumber("left pos motor units", leftPositionMotorUnits);
        SmartDashboard.putNumber("left pos meters", leftPositionMotorUnits / motorUnitsPerUnit);

        //update gyro based on positions
        gyro.update(leftPositionMotorUnits, rightPositionMotorUnits); //will update the return value of getCurrentHeading()

        //update position of the robot
        positionTracker.update(distanceDrivenMotorUnits, gyro.getHeading());
        return getCurrentPositionAndHeading();
    }

    /**
     * Calculates the acceleration of the robot.
     * @param percentOut The percent output of the motors.
     * @param initialVelocity The linear initial velocity of the robot.
     * @param braking True if the robot is braking, false otherwise.
     * @param useInternalFriction True if internal friction should be considered, false otherwise.
     * @return The acceleration of the robot in meters per second per second.
     */
    private double calculateAcceleration(double percentOut, double initialVelocity, boolean braking) {
        /**
         * Acceleration Equation:
         * a = (A * V0 * B * C + (p * n * Tm0)) * i / (r * m)
         * 
         * Where:
         * p = percent output of motors
         * n = number of motors per side of robot.
         * A = torque curve slope of motor
         * B = motor units per unit value
         * v0 = initial linear velocity of robot
         * C = 60 seconds per minute
         * Tm0 = initial torque of robot
         * r = radius of wheels being driven by motors
         * m = mass of robot
         * i = gear ratio of gearbox
         */

        if(braking) { //calculate acceleration if the robot is braking (a little different because the motor works differently)
            double brakingAcceleration = -1 * (
                calculateAcceleration(1, 0, false) - calculateAcceleration(1, initialVelocity, false)
            );
            
            double nonbrakingAcceleration = calculateAcceleration(percentOut, initialVelocity, false);
            double accel = brakingAcceleration + nonbrakingAcceleration;

            accel = accelerationWithFriction(accel, initialVelocity);
            return accel;
        }

        double slope = motor.getTorqueCurveSlope()
                    * initialVelocity
                    * motorUnitsPerUnit
                    * 60.0; //seconds per minute

        double top = (slope + (percentOut * motorsPerSide * motor.getInitialTorque())) * gearRatio;
        double bottom = wheelRadius * robotMass;

        double acceleration = top / bottom;
        acceleration = accelerationWithFriction(acceleration, initialVelocity);
        return acceleration;
    }

    /**
     * Accounts for gearbox friction.
     * @param acceleration A frictionless acceleration of the robot.
     * @param velocity Velocity of the robot.
     * @return New acceleration with friction accounted for.
     */
    private double accelerationWithFriction(double acceleration, double velocity) {
        if(Math.abs(velocity) < 0.01) {
            return acceleration;
        }

        //because friction moves opposite of motion, add some logic
        if(velocity > 0) {
            acceleration -= HyperdriveConstants.INTERNAL_FRICTION_ACCELERATION;
        } else {
            acceleration += HyperdriveConstants.INTERNAL_FRICTION_ACCELERATION;
        }

        return acceleration;
    }

    /**
     * Calculates the new velocity of a set of motors.
     * @param percentOut The percent output of the motors.
     * @param initialVelocity The linear initial velocity of the robot. 
     * @param timeInterval The time interval for which the robot drives.
     * @return The new linear velocity of the set of motors.
     */
    private double calculateNewVelocity(double percentOut, double initialVelocity, double timeInterval, boolean braking) {
        /**
         * Velocity Equation:
         * v = v0 + at
         * 
         * Where:
         * v = new velocity
         * v0 = initial velocity
         * a = acceleration
         * t = time interval
         */

        double acceleration = calculateAcceleration(percentOut, initialVelocity, braking);
        double velocity = initialVelocity + (acceleration * timeInterval);
        return velocity;
    }

    /**
     * Calculates the theoretical percent output of the motors to maintain a certain velocity.
     * This method is essentially an invert of the {@link #calculateNewVelocity(double, double, double)} method.
     * @param finalVelocity The final velocity (or the one being maintained)
     * @param initialVelocity The initial velocity
     * @param timeInterval The time interval
     * @return The theoretical percent output needed to maintain finalVelocity.
     */
    private double calculatePercentOutput(double velocity, double timeInterval) {
        /**
         * Percent output equation:
         * p = -(A * v * B * C * i) / (n * Tm0)
         * 
         * Where:
         * A = slope of the motor torque curve
         * v = velocity of the robot
         * B = motor units per unit
         * C = 60 seconds per minute
         * n = number of motors per side of the robot
         * Tm0 = initial torque of motor
         * i = gear ratio
         */
        
        double 
            slopedTorque = -1 * (motor.getTorqueCurveSlope() * velocity * motorUnitsPerUnit * 60.0 * gearRatio),
            initialTorque = motorsPerSide * motor.getInitialTorque(),
            percentOutput = slopedTorque / initialTorque;

        //percent output must be between -1 and 1
        percentOutput = (percentOutput < -1 ? -1 : (percentOutput > 1 ? 1 : percentOutput));
        return percentOutput;
    }

    /**
     * Calculates the linear displacement of one side of the robot
     * given the percent output of the motors on that side.
     * @param percentOut The percent output of the motors driving the side
     * of the robot being calculated for.
     * @return The displacement of the side of the robot.
     */
    private double calculateLinearDisplacement(double percentOut, double initialVelocity, double timeInterval, boolean braking) {
        /**
         * Displacement equation:
         * dx = v0t + (1/2)at^2
         * 
         * Where:
         * v0 = initial velocity
         * t = time interval
         * a = acceleration
         */

        double acceleration = calculateAcceleration(percentOut, initialVelocity, braking);
        SmartDashboard.putNumber("Acceleration: ", acceleration);
        double v0t = initialVelocity * timeInterval;
        double atsq = 0.5 * acceleration * (timeInterval * timeInterval);
        double displacement = v0t + atsq;
        return displacement;
    }

    /**
     * Determines whether or not a motor is braking.
     * @param percentOutput The current percent output of the motor.
     * @param initialVelocity The velocity of the motor.
     * @param timeInterval The time interval to consider.
     * @return {@code true} if the motor is braking, {@code false} otherwise.
     */
    private boolean isBraking(double percentOutput, double initialVelocity, double timeInterval) {
        //calculate theoretical percent outputs to determine whether or not to brake
        double theoreticalPercentOutput = calculatePercentOutput(initialVelocity, timeInterval);
        
        if(theoreticalPercentOutput > 0) {
            return percentOutput < theoreticalPercentOutput;
        } else {
            return percentOutput > theoreticalPercentOutput;
        }
    }
}
