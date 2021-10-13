// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Point2D;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandSetRobotPosition extends InstantCommand {
  SubsystemDrive drivetrain;
  
  public InstantCommandSetRobotPosition(SubsystemDrive drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double 
      x = HyperdriveUtil.getAndSetDouble("X", 0),
      y = HyperdriveUtil.getAndSetDouble("Y", 0),
      h = HyperdriveUtil.getAndSetDouble("H", 0);

    drivetrain.setPositionAndHeading(new Point2D(x, y, h));
  }
}
