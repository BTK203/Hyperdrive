// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.emulation.PreferenceEmulationParams;
import frc.robot.util.hyperdrive.emulation.TankTrajectory;
import frc.robot.util.hyperdrive.emulation.Trajectory;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Units;

public class CyborgCommandEmulatePath extends CommandBase {
  private SubsystemDrive drivetrain;

  /** Creates a new CyborgCommandEmulatePath. */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double
      kP = HyperdriveUtil.getAndSetDouble("Drive Velocity kP", 0),
      kI = HyperdriveUtil.getAndSetDouble("Drive Velocity kI", 0),
      kD = HyperdriveUtil.getAndSetDouble("Drive Velocity kD", 0),
      kF = HyperdriveUtil.getAndSetDouble("Drive Velocity kF", 0),
      iZone = HyperdriveUtil.getAndSetDouble("Drive Velocity IZone", 0),
      outLimit = HyperdriveUtil.getAndSetDouble("Drive Velocity Output Limit", 1),
      ramp = HyperdriveUtil.getAndSetDouble("Drive PID Ramp", 0.1875);

    drivetrain.setPIDF(kP, kI, kD, kF, iZone, outLimit, ramp);

    IEmulateParams parameters = new PreferenceEmulationParams(Units.LENGTH.INCHES);
    drivetrain.getHyperdrive().loadPath(drivetrain.getHyperdrive().getRecordedPath(), parameters);
    drivetrain.getHyperdrive().performInitialCalculations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory trajectory = drivetrain.getHyperdrive().calculateNextMovements();
    TankTrajectory tankTrajectory = trajectory.getTankTrajectory(Constants.ROBOT_WHEELBASE_WIDTH).convertTime(Units.TIME.MINUTES);

    drivetrain.setVelocities(tankTrajectory.getLeftVelocity(), tankTrajectory.getRightVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.getHyperdrive().finishPath();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getHyperdrive().pathFinished();
  }
}
