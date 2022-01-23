// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.emulation.PreferenceEmulationParams;
import frc.robot.util.hyperdrive.emulation.TankTrajectory;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Point2D;
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
  private boolean driveRecordedPath;
  private String pathFile; //only used if driveRecordedPath is true
  private double[] deviances; //array used for tracking deviances throughout the Path
  private int currentPoint;
  private long startTime;

  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive, IEmulateParams parameters, String path) {
    this.drivetrain = drivetrain;
    this.hyperdrive = hyperdrive;
    this.parameters = parameters;
    pathFile = path;
    driveRecordedPath = pathFile.equals("");

    addRequirements(this.drivetrain);
  }

  /**
   * Creates a new CyborgCommandEmulatePath. This constructor overrides all defaults to the user-specified arguments.
   * @param drivetrain The drivetrain of the robot.
   * @param hyperdrive The robot's {@link Hyperdrive}.
   * @param parameters The parameters for Hyperdrive to use while driving the robot through the path.
   */
  public CyborgCommandEmulatePath(SubsystemDrive drivetrain, Hyperdrive hyperdrive, IEmulateParams parameters) {
    this(drivetrain, hyperdrive, parameters, "");
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
    Path path = (driveRecordedPath ? hyperdrive.getRecordedPath() : new Path(pathFile));
    hyperdrive.loadPath(path, parameters);
    hyperdrive.performInitialCalculations();

    //get ready for deviance tracking
    currentPoint = 0;
    deviances = new double[hyperdrive.getLoadedPath().getPoints().length];

    //start the timer
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double wheelBaseUnits = HyperdriveUtil.convertDistance(Constants.WHEEL_BASE_WIDTH, Units.LENGTH.METERS, Constants.LENGTH_UNITS);
    TankTrajectory trajectory = hyperdrive.calculateNextMovements()
                                  .getTankTrajectory(wheelBaseUnits)
                                  .convertTime(Units.TIME.MINUTES);
                                  
    //calculate percent outputs
    double
      leftPercent = trajectory.getLeftPercentOutput(drivetrain.getLeftVelocityMotorUnits()),
      rightPercent = trajectory.getRightPercentOutput(drivetrain.getRightVelocityMotorUnits());

    //set the percent outputs
    drivetrain.setPercentOutputs(leftPercent, rightPercent);

    //track the deviance if necessary
    if(hyperdrive.getCurrentPoint() != currentPoint) {
      Point2D 
        robotPosition = hyperdrive.getRobotPositionAndHeading(),
        targetPosition = hyperdrive.getLoadedPath().getPoints()[hyperdrive.getCurrentPoint()];
      
      deviances[hyperdrive.getCurrentPoint()] = HyperdriveUtil.getDeviance(robotPosition, targetPosition);
    }

    SmartDashboard.putNumber("Current point", hyperdrive.getCurrentPoint());
    SmartDashboard.putNumber("Total points", hyperdrive.getTotalPoints());

    SmartDashboard.putNumber("Raw Left Velocity", trajectory.getRawLeftVelocity());
    SmartDashboard.putNumber("Raw Right Velocity", trajectory.getRawRightVelocity());

    SmartDashboard.putNumber("Left percent", leftPercent);
    SmartDashboard.putNumber("Right percent", rightPercent);
    
    SmartDashboard.putNumber("Target left velocity", trajectory.getLeftVelocity());
    SmartDashboard.putNumber("Target right velocity", trajectory.getRightVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    hyperdrive.finishPath();

    //stop timer and report time
    long endTime = System.currentTimeMillis();
    int totalTimeMS = (int) (endTime - startTime);
    double totalTimeS = totalTimeMS / (double) 1000;
    DriverStation.reportWarning("PATH TIME: " + Double.valueOf(totalTimeS), false);

    //report path driving deviance results, starting with average deviance
    double averageDeviance = 0;
    for(int i=0; i<deviances.length; i++) {
      averageDeviance += deviances[i];
    }

    averageDeviance /= deviances.length;
    DriverStation.reportWarning("AVERAGE DEVIANCE: " + Double.valueOf(averageDeviance).toString(), false);

    //also report maximum deviance
    double maxDeviance = 0;
    for(int i=0; i<deviances.length; i++) {
      if(deviances[i] > maxDeviance) {
        maxDeviance = deviances[i];
      }
    }

    DriverStation.reportWarning("MAX DEVIANCE: " + Double.valueOf(maxDeviance).toString(), false);
    System.out.println();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hyperdrive.pathFinished();
  }
}
