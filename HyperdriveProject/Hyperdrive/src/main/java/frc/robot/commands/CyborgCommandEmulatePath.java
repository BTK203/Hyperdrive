// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.emulation.ConstantEmulationParams;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.emulation.PreferenceEmulationParams;
import frc.robot.util.hyperdrive.emulation.TankTrajectory;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Units;

/**
 * This is a sample Command that uses {@link Hyperdrive} to drive the robot through a path.
 * Teams may choose to use this command and simply replace {@code SubsystemDrive} with the 
 * name of their drivetrain class. Teams can also use this class as a guide to writing their
 * own class.
 */
public class CyborgCommandEmulatePath extends CommandBase {
  private SubsystemDrive drivetrain;
  private Hyperdrive hyperdrive;
  private IEmulateParams parameters;
  private PIDController 
    leftController,
    rightController;

  /**
   * Creates a new CyborgCommandEmulatePath. This constructor overrides all defaults to the user-specified arguments.
   * @param drivetrain The drivetrain of the robot.
   * @param hyperdrive The robot's {@link Hyperdrive}.
   * @param parameters The parameters for Hyperdrive to use while driving the robot through the path.
   */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive, IEmulateParams parameters) {
    this.drivetrain = drivetrain;
    this.hyperdrive = hyperdrive;
    this.parameters = parameters;

    addRequirements(this.drivetrain);
  }

  /**
   * Creates a new CyborgCommandEmulatePath using the default parameters to emulate the Path.
   * @param drivetrain The drivetrain of the robot.
   * @param hyperdrive The robot's {@link Hyperdrive}.
   */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive) {
    this(drivetrain, hyperdrive, new PreferenceEmulationParams(hyperdrive.getLengthUnits()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hyperdrive.loadPath(hyperdrive.getRecordedPath(), parameters);
    hyperdrive.performInitialCalculations();

    //get PID stuff
    double
      kp = HyperdriveUtil.getAndSetDouble("Drive kP", 0),
      ki = HyperdriveUtil.getAndSetDouble("Drive kI", 0),
      kd = HyperdriveUtil.getAndSetDouble("Drive kD", 0);

    leftController = new PIDController(kp, ki, kd);
    rightController = new PIDController(kp, ki, kd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TankTrajectory trajectory = hyperdrive.calculateNextMovements()
                                  .getTankTrajectory(Constants.WHEEL_BASE_WIDTH)
                                  .convertTime(Units.TIME.MINUTES)
                                  .invertTurn();

    //set appropriate setpoints
    leftController.setSetpoint(trajectory.getLeftVelocity());
    rightController.setSetpoint(trajectory.getRightVelocity());
    
    //calculate percent outputs
    double
      leftPercent = leftController.calculate(drivetrain.getLeftVelocityMotorUnits()),
      rightPercent = rightController.calculate(drivetrain.getRightVelocityMotorUnits());

    //set the percent outputs
    drivetrain.setPercentOutputs(leftPercent, rightPercent);

    SmartDashboard.putNumber("Current point", hyperdrive.getCurrentPoint());
    SmartDashboard.putNumber("Total points", hyperdrive.getTotalPoints());

    SmartDashboard.putNumber("Raw Left Velocity", trajectory.getRawLeftVelocity());
    SmartDashboard.putNumber("Raw Right Velocity", trajectory.getRawRightVelocity());

    SmartDashboard.putNumber("Left percent", leftPercent);
    SmartDashboard.putNumber("Right percent", rightPercent);

    SmartDashboard.putNumber("left error", leftController.getPositionError());
    SmartDashboard.putNumber("left velocity world units", drivetrain.getLeftVelocity());
    SmartDashboard.putNumber("left velocity motor units", hyperdrive.toMotorUnits(drivetrain.getLeftVelocity()));

    SmartDashboard.putNumber("Target left velocity", trajectory.getLeftVelocity());
    SmartDashboard.putNumber("Target right velocity", trajectory.getRightVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    hyperdrive.finishPath();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hyperdrive.pathFinished();
  }
}
