// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.hyperdrive.Hyperdrive;

/**
 * A sample command that records a {@link Path}. Teams can run this command
 * as-is on their robot and may choose to do so if they do not want advanced
 * functionality in their recording command. 
 * After the command completes, the recorded Path will be sent to PathVisualizer
 * for viewing.
 */
public class CyborgCommandRecordPath extends CommandBase {
  private Hyperdrive hyperdrive;

  /** 
   * Creates a new CyborgCommandRecordPath. 
   * @param hyperdrive The robot's {@link Hyperdrive}.
   */
  public CyborgCommandRecordPath(Hyperdrive hyperdrive) {
    this.hyperdrive = hyperdrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hyperdrive.initializeRecorder("src/main/java/frc/robot/points.txt");
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
