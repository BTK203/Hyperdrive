// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.KeyboardListener;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.enumeration.DriveStyle;
import frc.robot.util.hyperdrive.simulator.SimulatedMotor;
import frc.robot.util.hyperdrive.simulator.SimulatedRobot;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.Units;

/**
 * The test environment's drivetrain subsystem class. This class also serves as a simulator for 
 * testing. 
 */
public class SubsystemDrive extends SubsystemBase {
  //runtime values
  private double
    leftPercentOutput,
    rightPercentOutput;

  private double
    forwardInput,
    steeringInput;

  //time
  private double lastIterationTime;
  private long lastUpdateTime;

  //tools
  private SimulatedRobot simulatedRobot;
  private Hyperdrive hyperdrive;

  /** 
   * The SubsystemDrive of the test environment. Works with a Simulated Robot to 
   * simulate actual driving.
   */
  public SubsystemDrive() {
    leftPercentOutput = 0;
    rightPercentOutput = 0;

    lastUpdateTime = System.currentTimeMillis();

    //get motor units per unit value, but convert it to motor units per meter (instead of inch)
    double mupuConversionFactor = HyperdriveUtil.convertDistance(1, Units.LENGTH.INCHES, Units.LENGTH.METERS);
    double mupuMeters = 0.472 / mupuConversionFactor;

    //define hyperdrive
    hyperdrive = new Hyperdrive(DriveStyle.TANK, Units.LENGTH.METERS, mupuMeters, Units.FORCE.POUND, 110);

    //define simulated robot
    simulatedRobot = new SimulatedRobot(
      SimulatedMotor.NEO, 
      2,
      HyperdriveUtil.convertDistance(6, Units.LENGTH.INCHES, Units.LENGTH.METERS), 
      HyperdriveUtil.convertDistance(20, Units.LENGTH.INCHES, Units.LENGTH.METERS), 
      7.14,
      110, 
      Units.FORCE.POUND, 
      mupuMeters
    );

    KeyboardListener.start();
  }

  @Override
  public void periodic() {
    //get change in time
    long currentTime = System.currentTimeMillis();
    long deltaTime = currentTime - lastUpdateTime;
    double deltaTimeSeconds = deltaTime / 1000.0; //1000 ms in a second
    lastIterationTime = deltaTimeSeconds;
    lastUpdateTime = currentTime;

    //update simulated robot
    Point2D currentPositionAndHeading = simulatedRobot.update(leftPercentOutput, rightPercentOutput, deltaTimeSeconds);

    //update Hyperdrive
    hyperdrive.update(currentPositionAndHeading);
  }

  /**
   * Drives the simulated robot using input from the keyboard.
   * Keyboard input will only be captured if the key listener window is focused.
   */
  public void driveWithKeyboardInput() {
    double increment = lastIterationTime * Constants.KEY_THROTTLE_PER_SECOND; //unit; percent

    //forward input
    if(KeyboardListener.getWPressed()) {
      forwardInput += increment;
    } else if(KeyboardListener.getSPressed()) {
      forwardInput -= increment;
    } else {
      //gravitate to 0
      forwardInput += (forwardInput < 0 ? increment : -1 * increment);
      if(forwardInput < increment) {
        forwardInput = 0;
      }
    }

    //steering input
    if(KeyboardListener.getAPressed()) {
      steeringInput += increment;
    } else if(KeyboardListener.getDPressed()) {
      steeringInput -= increment;
    } else {
      //gravitate to 0
      steeringInput += (steeringInput < 0 ? increment : -1 * increment);
      if(steeringInput < increment) {
        steeringInput = 0;
      }
    }

    //make sure that forwardInput and steeringInput are between -1 and 1
    forwardInput = (forwardInput < -1 ? -1 : (forwardInput > 1 ? 1 : forwardInput));
    steeringInput = (steeringInput < -1 ? -1 : (steeringInput > 1 ? 1 : steeringInput));

    //calculate new percent outputs based on forward and steer inputs
    double left = forwardInput - steeringInput;
    double right = forwardInput + steeringInput;

    //make sure that left and right are between -1 and 1
    left = (left < -1 ? -1 : (left > 1 ? 1 : left));
    right = (right < -1 ? -1 : (right > 1 ? 1 : right));

    SmartDashboard.putNumber("left", left);
    SmartDashboard.putNumber("right", right);

    //set left and right
    setPercentOutputs(left, right);
  }

  /**
   * Returns the drivetrain's {@link Hyperdrive}.
   * @return The drivetrain's Hyperdrive tool.
   */
  public Hyperdrive getHyperdrive() {
    return hyperdrive;
  }

  /**
   * Sets the percent outputs of the robot.
   * @param left The left percent output
   * @param right The right percent output.
   */
  public void setPercentOutputs(double left, double right) {
    leftPercentOutput = left;
    rightPercentOutput = right;
  }

  /**
   * Returns the left velocity of the robot.
   * @return Linear velocity of the robot's left wheels.
   */
  public double getLeftVelocity() {
    return simulatedRobot.getLeftVelocity();
  }

  /**
   * Returns the right velocity of the robot.
   * @return Linear velocity of the robot's right wheels.
   */
  public double getRightVelocity() {
    return simulatedRobot.getRightVelocity();
  }

  /**
   * Returns the left velocity of the robot in motor units.
   * @return Left velocity of robot in motor units
   */
  public double getLeftVelocityMotorUnits() {
    double leftVelocity = getLeftVelocity(); //currently in meters per second
    leftVelocity *= 60; //meters per minute
    leftVelocity = hyperdrive.toMotorUnits(leftVelocity);

    return leftVelocity;
  }

  /**
   * Returns the right velocity of the robot in motor units.
   * @return Right velocity of the robot in motor units
   */
  public double getRightVelocityMotorUnits() {
    double rightVelocity = getRightVelocity();
    rightVelocity *= 60;
    rightVelocity = hyperdrive.toMotorUnits(rightVelocity);

    return rightVelocity;
  }

  /**
   * Zeros the Hyperdrive position. Note that because the robot 
   */
  public void zeroPositionAndHeading() {
    hyperdrive.zeroPositionAndHeading(true);
    simulatedRobot.zeroPositionAndHeading();
  }

  /**
   * Sets the position and heading of the robot.
   * @param positionAndHeading new position and heading.
   */
  public void setPositionAndHeading(Point2D positionAndHeading) {
    simulatedRobot.setCurrentPositionAndHeading(positionAndHeading);
  }

  /**
   * Sets the robot's position and heading to the start of the most recently recorded path.
   */
  public void setPoseToRecordedPathStart() {
    if(hyperdrive.getRecordedPath().isValid()) { //if the most recently recorded path exists...
      Point2D pathStart = hyperdrive.getRecordedPath().getPoints()[0];
      simulatedRobot.setCurrentPositionAndHeading(pathStart);
    }
  }

  /**
   * Stops the drivetrain motors.
   */
  public void stop() {
    setPercentOutputs(0, 0);
  }

}
