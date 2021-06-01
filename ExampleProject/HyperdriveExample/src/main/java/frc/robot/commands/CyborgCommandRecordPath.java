// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.Hyperdrive;

public class CyborgCommandRecordPath extends CommandBase {
  private Hyperdrive hyperdrive;

  /** Creates a new CyborgCommandRecordPath. */
  public CyborgCommandRecordPath(SubsystemDrive drivetrain) {
    hyperdrive = drivetrain.getHyperdrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hyperdrive.initializeRecorder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hyperdrive.stopRecorder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
