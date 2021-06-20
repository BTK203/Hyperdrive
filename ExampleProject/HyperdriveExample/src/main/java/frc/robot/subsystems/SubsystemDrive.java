// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Xbox;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Units;

public class SubsystemDrive extends SubsystemBase {
  private CANSparkMax 
    leftMaster,
    leftSlave,
    rightMaster,
    rightSlave;

  private AHRS navX; //navX object
  private Hyperdrive hyperdrive;

  /** Creates a new SubsystemDrive. */
  public SubsystemDrive() {
    //define motor controllers
    leftMaster = new CANSparkMax(Constants.LEFT_MASTER_ID, MotorType.kBrushless);
    leftSlave  = new CANSparkMax(Constants.LEFT_SLAVE_ID, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_ID, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.RIGHT_SLAVE_ID, MotorType.kBrushless);

    //define navx and hyperdrive
    navX = new AHRS(Port.kUSB);
    hyperdrive = new Hyperdrive(Units.LENGTH.INCHES, Constants.MOTOR_ROTATIONS_PER_INCH, Units.FORCE.POUND, 120);

    configureMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double 
      leftPosition = leftMaster.getEncoder().getPosition(), //position of left motors
      rightPosition = rightMaster.getEncoder().getPosition(), //position of right motors
      heading = navX.getAngle(); //heading of robot

    hyperdrive.update(leftPosition, rightPosition, heading);

    SmartDashboard.putNumber("Left Velocity", leftMaster.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Velocity", rightMaster.getEncoder().getVelocity());
  }

  /**
   * Returns the drivetrain's Hyperdrive.
   * @return Hyperdrive.
   */
  public Hyperdrive getHyperdrive() {
    return hyperdrive;
  }

  /**
   * Drives the robot using human controller input
   * @param joystick The controller to use.
   */
  public void driveManually(Joystick joystick) {
    double throttle = Xbox.RT(joystick) - Xbox.LT(joystick);
    double steering = Xbox.LEFT_X(joystick);

    double leftDrive  = throttle + steering;
    double rightDrive = throttle - steering;

    //make sure leftDrive and rightDrive are between -1 and 1
    leftDrive  = ( leftDrive < -1 ? -1 : ( leftDrive > 1 ? 1 : leftDrive ) );
    rightDrive = ( rightDrive < -1 ? -1 : ( rightDrive > 1 ? 1 : rightDrive ) );

    leftMaster.set(leftDrive);
    leftSlave.set(leftDrive);
    rightMaster.set(rightDrive);
    rightSlave.set(rightDrive);
  }

  /**
   * Zeroes the drivetrain. This includes encoders, gyro, and Hyperdrive
   */
  public void zeroDrivetrain() {
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);
    navX.zeroYaw();
    hyperdrive.zeroPositionAndHeading(true);
  }

  /**
   * Sets the PIDF constants of the drivetrain motors.
   * @param p The kP to set.
   * @param i The kI to set.
   * @param d The kD to set.
   * @param f The kF to set.
   * @param izone The izone to set.
   * @param outLimit The maximum allowed output.
   * @param ramp The closed loop ramp rate, in seconds, to set.
   */
  public void setPIDF(double p, double i, double d, double f, double izone, double outLimit, double ramp) {
    //set constants on left motor controller
    leftMaster.getPIDController().setP(p);
    leftMaster.getPIDController().setI(i);
    leftMaster.getPIDController().setD(d);
    leftMaster.getPIDController().setFF(f);
    leftMaster.getPIDController().setIZone(izone);
    leftMaster.getPIDController().setOutputRange(-1 * outLimit, outLimit);
    leftMaster.setClosedLoopRampRate(ramp);

    //set constants on right motor controller
    rightMaster.getPIDController().setP(p);
    rightMaster.getPIDController().setI(i);
    rightMaster.getPIDController().setD(d);
    rightMaster.getPIDController().setFF(f);
    rightMaster.getPIDController().setIZone(izone);
    rightMaster.getPIDController().setOutputRange(-1 * outLimit, outLimit);
    rightMaster.setClosedLoopRampRate(ramp);
  }

  /**
   * Sets the velocities of the left and right drivetrain motors.
   * @param leftVelocity The target velocity of the left motors, in RPM.
   * @param rightVelocity The target velocity of the right motors, in RPM.
   */
  public void setVelocities(double leftVelocity, double rightVelocity) {
    leftMaster.getPIDController().setReference(leftVelocity, ControlType.kVelocity);
    rightMaster.getPIDController().setReference(rightVelocity, ControlType.kVelocity);
  }

  /**
   * Sets the output of the motors to 0.
   */
  public void stop() {
    leftMaster.set(0);
    rightMaster.set(0);
  }

  /**
   * Configures the motors.
   */
  private void configureMotors() {
    //set amp limits
    leftMaster.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
    leftSlave.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
    rightMaster.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
    rightSlave.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);

    //set motor inverts
    leftMaster.setInverted(Constants.LEFT_MASTER_INVERT);
    leftSlave.setInverted(Constants.LEFT_SLAVE_INVERT);
    rightMaster.setInverted(Constants.RIGHT_MASTER_INVERT);
    rightSlave.setInverted(Constants.RIGHT_SLAVE_INVERT);

    //set ramp rate
    double rampRate = HyperdriveUtil.getAndSetDouble("Drive Ramp Rate", 0.25);
    leftMaster.setOpenLoopRampRate(rampRate);
    leftSlave.setOpenLoopRampRate(rampRate);
    rightMaster.setOpenLoopRampRate(rampRate);
    rightSlave.setOpenLoopRampRate(rampRate);

    //set the slave motors as followers of the master motors.
    //as followers, the slave motors will automatically be set to whatever output the master motors are giving.
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }
}
