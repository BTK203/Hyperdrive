// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.emulation.ConstantEmulationParams;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.emulation.TankTrajectory;
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
  private Path path;
  private IEmulateParams parameters;

  /**
   * Creates a new CyborgCommandEmulatePath. This constructor overrides all defaults to the user-specified arguments.
   * @param drivetrain The drivetrain of the robot.
   * @param hyperdrive The robot's {@link Hyperdrive}.
   * @param path The Path that the robot will drive.
   * @param parameters The parameters for Hyperdrive to use while driving the robot through the path.
   */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive, Path path, IEmulateParams parameters) {
    this.drivetrain = drivetrain;
    this.hyperdrive = hyperdrive;
    this.path = path;
    this.parameters = parameters;

    addRequirements(this.drivetrain);
  }

  /**
   * Creates a new CyborgCommandEmulatePath using the default parameters to emulate the Path.
   * @param drivetrain The drivetrain of the robot.
   * @param hyperdrive The robot's {@link Hyperdrive}.
   * @param path The Path that the robot will drive.
   */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive, Path path) {
    this(drivetrain, hyperdrive, path, ConstantEmulationParams.getDefaults(hyperdrive.getDistanceUnits()));
  }

  /**
   * Creates a new CyborgCommandEmulatePath using the last Path that was recorded, and the default parameters to drive it.
   * @param drivetrain The drivetrain of the robot.
   * @param hyperdrive The robot's {@link Hyperdrive}.
   */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive) {
    this(drivetrain, hyperdrive, new Path(HyperdriveConstants.PATH_RECORDER_DEFAULT_RECORD_PATH));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hyperdrive.loadPath(path, parameters);
    hyperdrive.specifyResultsFile("D:/_Users/Brach/Projects/FRC/Hyperdrive/Hyperdrive/src/main/java/frc/robot/results.txt"); //for test purposes
    hyperdrive.performInitialCalculations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TankTrajectory trajectory = hyperdrive.calculateNextMovements().getTankTrajectory(Constants.WHEEL_BASE_WIDTH).convertTime(Units.TIME.MINUTES);
    drivetrain.setLeftVelocity(trajectory.getLeftVelocity());
    drivetrain.setRightVelocity(trajectory.getRightVelocity());

    SmartDashboard.putNumber("Current point", hyperdrive.getCurrentPoint());
    SmartDashboard.putNumber("Total points", hyperdrive.getTotalPoints());

    SmartDashboard.putNumber("Raw Left Velocity", trajectory.getRawLeftVelocity());
    SmartDashboard.putNumber("Raw Right Velocity", trajectory.getRawRightVelocity());

    SmartDashboard.putNumber("Target left velocity", trajectory.getLeftVelocity());
    SmartDashboard.putNumber("Target right velocity", trajectory.getRightVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    hyperdrive.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hyperdrive.pathFinished();
  }
}
