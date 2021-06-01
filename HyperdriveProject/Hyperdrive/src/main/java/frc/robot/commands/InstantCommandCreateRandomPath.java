// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.recording.PathRecorder;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.Units;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandCreateRandomPath extends InstantCommand {
  private PathRecorder recorder;
  private Hyperdrive hyperdrive;
  private Point2D currentPoint;

  public InstantCommandCreateRandomPath(String destination, Hyperdrive hyperdrive) {
    this.recorder = new PathRecorder(destination, Units.LENGTH.INCHES);
    this.hyperdrive = hyperdrive;
    this.recorder.init();
    this.currentPoint = new Point2D(0, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(int i=0; i<75; i++) {
      double currentHeading = currentPoint.getHeading();
      currentHeading += (Math.random() * 16) - 4;
      double newHeadingRadians = Math.toRadians(currentHeading);
      double displacmentX = 5 * Math.cos(newHeadingRadians);
      double displacmentY = 5 * Math.sin(newHeadingRadians);
      Point2D newCurrentPosition = new Point2D(currentPoint.getX() + displacmentX, currentPoint.getY() + displacmentY, currentHeading);
      currentPoint = newCurrentPosition;
      recorder.recordPoint(currentPoint);
    }

    recorder.closeFile();
    hyperdrive.sendPath(new Path(recorder.getFilePath()), "Randomly Generated Path");
  }
}
