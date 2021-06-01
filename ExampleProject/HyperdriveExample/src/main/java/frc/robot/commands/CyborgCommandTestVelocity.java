// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;

public class CyborgCommandTestVelocity extends CommandBase {
  private SubsystemDrive drivetrain;

  /** Creates a new CyborgCommandTestVelocity. */
  public CyborgCommandTestVelocity(SubsystemDrive drivetrain) {
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
      outLimit = HyperdriveUtil.getAndSetDouble("Drive Velocity Output Limit", 1);

    drivetrain.setPIDF(kP, kI, kD, kF, iZone, outLimit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocityRPM = HyperdriveUtil.getAndSetDouble("Drive Test Velocity", 1000);
    drivetrain.setVelocities(velocityRPM, velocityRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
