// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.TankGyro;

public class SubsystemDrive extends SubsystemBase {
  private CANSparkMax
    leftMaster,
    leftSlave,
    rightMaster,
    rightSlave;
  
  private double
    leftPosition,
    rightPosition,
    heading;

  private Hyperdrive hyperdrive;
  private TankGyro gyro;

  /** Creates a new SubsystemDrive. */
  public SubsystemDrive() {
    leftMaster  = new CANSparkMax(Constants.LEFT_MASTER_ID, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_ID, MotorType.kBrushless);
    leftSlave   = new CANSparkMax(Constants.LEFT_SLAVE_ID, MotorType.kBrushless);
    rightSlave  = new CANSparkMax(Constants.RIGHT_SLAVE_ID, MotorType.kBrushless);

    leftPosition = 0;
    rightPosition = 0;
    heading = 0;

    hyperdrive = new Hyperdrive();
    gyro = new TankGyro(24, 0.472);

    setFollowers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hyperdrive.update(leftPosition, rightPosition, heading);
    gyro.update(leftPosition, rightPosition);
    heading = gyro.getHeading();

    SmartDashboard.putNumber("Left Position", leftPosition);
    SmartDashboard.putNumber("Right Position", rightPosition);
    SmartDashboard.putNumber("Heading", heading);
  }

  /**
   * Returns the drivetrain's {@link Hyperdrive}.
   * @return
   */
  public Hyperdrive getHyperdrive() {
    return hyperdrive;
  }

  /**
   * Zeros the Hyperdrive position.
   */
  public void zeroPositionAndHeading() {
    hyperdrive.zeroPositionAndHeading(true);
  }

  /**
   * Zeros the tank gyro, waiting for encoders before it does such.
   */
  public void zeroGyro() {
    gyro.zeroHeading(true);
  }

  /**
   * Zeros the position readouts of the motors.
   */
  public void zeroMotorPositions() {
    leftPosition = 0;
    rightPosition = 0;
    heading = 0;
  }

  /**
   * Sets random stuff
   */
  public void randomize() {
    leftPosition = (Math.random() * 100) - 50;
    rightPosition = (Math.random() * 100) - 50;
    heading = (Math.random() * 360) - 180;

    double x = (Math.random() * 100);
    double y = (Math.random() * 100);
    hyperdrive.setPositionAndHeading(new Point2D(x, y, heading));
  }

  /**
   * Displaces the robot in a random direction.
   */
  public void displaceRandomly() {
    leftPosition += (Math.random() * 8) + 2;
    rightPosition += (Math.random() * 8) + 2;
    heading += (Math.random() * 40) - 20;
  }

  /**
   * Displaces the drivetrain
   * @param leftDisplacement Amount to add to left position
   * @param rightDisplacement Amount to add to right displacement
   */
  public void displace(double leftDisplacement, double rightDisplacement) {
    leftPosition += leftDisplacement;
    rightPosition += rightDisplacement;
  }

  /**
   * Sets the heading.
   * @param heading new heading
   */
  public void setAbsoluteHeading(double heading) {
    this.heading = heading;
  }
  
  /**
   * Sets the position
   * @param position new position
   */
  public void setPosition(Point2D position) {
    hyperdrive.setPositionAndHeading(position);
    this.heading = position.getHeading();
  }

  /**
   * Sets the PID constants of the master controllers.
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param f Feed-Forward
   * @param izone Margin of error, inside which the I will take effect.
   * @param outLimit Maximum allowed output.
   */
  public void setPIDF(double p, double i, double d, double f, double izone, double outLimit) {
    leftMaster.getPIDController().setP(p);
    leftMaster.getPIDController().setI(i);
    leftMaster.getPIDController().setD(d);
    leftMaster.getPIDController().setFF(f);
    leftMaster.getPIDController().setIZone(izone);
    leftMaster.getPIDController().setOutputRange(outLimit * -1, outLimit);

    rightMaster.getPIDController().setP(p);
    rightMaster.getPIDController().setI(i);
    rightMaster.getPIDController().setD(d);
    rightMaster.getPIDController().setFF(f);
    rightMaster.getPIDController().setIZone(izone);
    rightMaster.getPIDController().setOutputRange(outLimit * -1, outLimit);
  }

  /**
   * Sets the target velocity of the left motors.
   * @param leftVelocity PID velocity target of left motors.
   */
  public void setLeftVelocity(double leftVelocity) {
    leftMaster.getPIDController().setReference(leftVelocity, ControlType.kVelocity);
  }

  /**
   * Sets the target velocity of the right motors.
   * @param rightVelocity PID velocity target of right motors.
   */
  public void setRightVelocity(double rightVelocity) {
    rightMaster.getPIDController().setReference(rightVelocity, ControlType.kVelocity);
  }

  /**
   * Stops the drivetrain motors.
   */
  public void stop() {
    leftMaster.set(0);
    leftSlave.set(0);
    rightMaster.set(0);
    rightSlave.set(0);
  }

  /**
   * Sets the slave motors as "following" the master motors.
   * This means that PID and output demands only need to be sent
   * to the master motors, and the slave motors will simply match their output.
   */
  private void setFollowers() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }
}
