// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.simulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        rightVelocity;

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
     * Calculates the new position of the robot if it was driven for a certain time interval.
     * @param leftPercentOutput The percent output of the left motors.
     * @param rightPercentOutput The percent output of the right motors.
     * @param timeInterval The time interval for which the robot drives.
     * @return The position of the robot after it drives for the specified time interval
     */
    public Point2D update(double leftPercentOutput, double rightPercentOutput, double timeInterval) {
        double
            leftDisplacement = calculateLinearDisplacement(leftPercentOutput, leftVelocity, timeInterval),
            rightDisplacement = calculateLinearDisplacement(rightPercentOutput, rightVelocity, timeInterval),
            distanceDriven = (leftDisplacement + rightDisplacement) / 2;

        SmartDashboard.putNumber("leftPercent", leftPercentOutput);
        SmartDashboard.putNumber("leftVelocity", leftVelocity);
        SmartDashboard.putNumber("leftDisplacement", leftDisplacement);
        SmartDashboard.putNumber("timeInterval", timeInterval);

        //update velocities
        leftVelocity = calculateNewVelocity(leftPercentOutput, leftVelocity, timeInterval);
        rightVelocity = calculateNewVelocity(rightPercentOutput, rightVelocity, timeInterval);

        //update motor positions
        leftPositionMotorUnits += leftDisplacement * motorUnitsPerUnit;
        rightPositionMotorUnits += rightDisplacement * motorUnitsPerUnit;

        //update gyro based on positions
        gyro.update(leftPositionMotorUnits, rightPositionMotorUnits); //will update the return value of getCurrentHeading()

        //update position of the robot
        positionTracker.update(distanceDriven, getCurrentPositionAndHeading().getHeading());
        return getCurrentPositionAndHeading();
    }

    /**
     * Calculates the new velocity of a set of motors.
     * @param percentOut The percent output of the motors.
     * @param initialVelocity The linear initial velocity of the robot. 
     * @param timeInterval The time interval for which the robot drives.
     * @return The new linear velocity of the set of motors.
     */
    private double calculateNewVelocity(double percentOut, double initialVelocity, double timeInterval) {
        //only takes into account the slope of the torque curve.
        double sloped = percentOut
                    * motorsPerSide
                    * motor.getTorqueCurveSlope()
                    * motorUnitsPerUnit
                    * 60 //seconds per minute
                    * initialVelocity
                    * timeInterval; //Unit of entire value: Nms

        //only using the y-intercept of the torque curve.
        double intercepted = motor.getInitialTorque() * timeInterval; //Unit: Nms

        //radius of wheels, mass of robot, gear ratio.
        double rmg = wheelRadius * robotMass * gearRatio; //Unit: kgm

        //new velocity
        double newVelocity = ((sloped + intercepted) / rmg) + initialVelocity; //m/s
        return newVelocity;
    }

    /**
     * Calculates the linear displacement of one side of the robot
     * given the percent output of the motors on that side.
     * @param percentOut The percent output of the motors driving the side
     * of the robot being calculated for.
     * @return The displacement of the side of the robot.
     */
    private double calculateLinearDisplacement(double percentOut, double initialVelocity, double timeInterval) {
        //only taking into account the slope of the torque curve
        double sloped = percentOut
                    * motorsPerSide
                    * motor.getTorqueCurveSlope()
                    * motorUnitsPerUnit
                    * 60 // seconds per minute
                    * initialVelocity
                    * Math.pow(timeInterval, 2); //Nms^2

        //only using y-intercept of the torque curve
        double intercepted = motor.getInitialTorque() * Math.pow(timeInterval, 2); //Nms^2

        //radius of wheels, mass of robot, gear ratio, all multiplied together and by 2
        double rmg = 2 * wheelRadius * robotMass * gearRatio; //kgm

        //displacement of robot with initial velocity
        double displacementWithInitial = initialVelocity * timeInterval; //m

        //linear displacement
        double linearDisplacement = ((sloped + intercepted) / rmg) + displacementWithInitial; // m
        return linearDisplacement;
    }
}
