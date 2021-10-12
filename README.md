![logo](https://github.com/BTK203/InfiniteRecharge-2021/blob/master/banner.png?raw=true)
# Hyperdrive
An easy-to-implement autonomous driving library for FRC robots, developed by FRC team 3695, Foximus Prime. Hyperdrive was able to produce a cumulative time of
28.3 seconds on the Auto-Nav challenge in the 2021 FRC season, and a 9.9 second cumulative time on the Galactic Search challenge. It is currently being developed to support tank-style robots, but support for swerve robots will hopefully be implemented as well.

The actual library itself is located inside of its testing environment at [HyperdriveProject/Hyperdrive/src/main/java/frc/robot/util/hyperdrive](https://github.com/BTK203/Hyperdrive/tree/develop/HyperdriveProject/Hyperdrive/src/main/java/frc/robot/util/hyperdrive).

**Check out the class and method reference at [https://btk203.github.io/Hyperdrive/](https://btk203.github.io/Hyperdrive/)!**

## Prerequisites
In order for Hyperdrive to work on your robot, it must meet these requirements:

- It must have a tank-style drivetrain. Unfortunately, Hyperdrive does not yet support mecanum or swerve robots.
- It should have a physical gyro. Hyperdrive does provide a TankGyro class which can be used in prototyping and simulation cases, but it is not recommended on competition robots.

## Installation
To install Hyperdrive, simply extract the hyperdrive folder from the [latest release](https://github.com/BTK203/Hyperdrive/releases/latest) into a folder in your robot project named "util".

![logo](https://github.com/BTK203/Hyperdrive/blob/develop/images/hyperdrive-install.png?raw=true)

## Implementation

NOTE: This readme provides basic instructions for installing and using Hyperdrive. For a step-by-step tutorial, see the [Hyperdrive manual](https://github.com/BTK203/Hyperdrive/blob/main/Hyperdrive%20Manual.pdf).

### Measuring needed values
Hyperdrive needs two values to work on your robot: A scalar to convert motor rotations to linear positions, and the wheelbase width of your robot.

**To measure your motor unit scalar**, use a tape measure to drive your robot a known distance, such as 10 feet. Then, calculate how much your motor positions changed. Then divide the change in your motor positions by the distance that the robot drove. For example, if your motors start at 0 rotations and end at 49.44 rotations after the robot drives 120 inches, your value would be:

![logo](https://github.com/BTK203/Hyperdrive/blob/develop/images/mupu-equation.png?raw=true)

**To measure your robot's wheelbase width**, simply use a tape measure to find the distance between your robot's wheels:

![logo](https://github.com/BTK203/Hyperdrive/blob/develop/images/wheelbase.png?raw=true)

Remember to keep units consistent! If you used inches to determine your motor units scalar, measure the wheelbase width in inches!

After these values are measured, save them somewhere safe, like the Constants class in your robot project.

### Adding Hyperdrive to your code
To add Hyperdrive to your code, simply add a Hyperdrive object to your drivetrain class, then call update() in your periodic() method:

```java
  private Hyperdrive hyperdrive; // Hyperdrive object

  /** Creates a new SubsystemDrive. */
  public SubsystemDrive() {
    hyperdrive = new Hyperdrive(
      DriveStyle.TANK,                      // Drive Style
      Units.LENGTH.METERS,                  // Length Unit
      Constants.MOTOR_ROTATIONS_PER_METER,  // Motor Units per Unit scalar
      Units.FORCE.POUND,                    // Weight Unit
      Constants.ROBOT_WEIGHT                // Robot Weight
    );
  }
  ```

```java
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double
      leftPosition  = leftMain.getEncoder().getPosition(), //get left position
      rightPosition = rightMain.getEncoder().getPosition(), //get right position
      heading       = gyro.getAngle(); //get yaw angle

    hyperdrive.update(leftPosition, rightPosition, heading);
  }
  ```

For ease of access, add an accessor for the Hyperdrive object:

```java
  /**
   * Returns the drivetrain's Hyperdrive object.
   */
  public Hyperdrive getHyperdrive() {
    return hyperdrive;
  }
```

Make sure your drivetrain has public methods for setting percent outputs and getting motor velocities.

### Zeroing
Hyperdrive keeps track of the robot's 2-D position on the field. At times, you may want to zero it. This is an example of a method in the drivetrain that would zero the motors, gyro, and Hyperdrive:

```java
  /**
   * Zeros the motors, the gyro, and Hyperdrive.
   */
  public void zero() {
    leftMain.getEncoder().setPosition(0);
    rightMain.getEncoder().setPosition(0);
    gyro.zeroYaw();
    hyperdrive.zeroPositionAndHeading(true);
  }
```

## Using Hyperdrive to Drive Your Robot
The following commands are templates that you can use to get Hyperdrive driving your robot around. However, you may need to change some names and method calls.
In these examples, `SubsystemDrive` refers to some drivetrain subsystem, and `getLeftVelocityMotorUnits()` (and its right counterpart) is a method that directly returns the velocity of the left motor encoder.

### Recording Paths
Use this simple Command to record paths
```java
public class RecordPath extends CommandBase {
  private Hyperdrive hyperdrive;

  /** Creates a new RecordPath. */
  public RecordPath(SubsystemDrive drivetrain) {
    this.hyperdrive = drivetrain.getHyperdrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hyperdrive.initializeRecorder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {} //nothing needs to go here, Hyperdrive records a point every time it is updated

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
```

### Emulating Paths
Use this Command to emulate paths.
```java
public class EmulatePath extends CommandBase {
  private SubsystemDrive drivetrain;

  /** Creates a new EmulatePath. */
  public EmulatePath(SubsystemDrive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //create parameters and load path
    IEmulateParams emulateParameters = new PreferenceEmulationParams(drivetrain.getHyperdrive().getLengthUnits());
    drivetrain.getHyperdrive().loadPath(drivetrain.getHyperdrive().getRecordedPath(), emulateParameters);

    //perform calculations vital to Hyperdrive's performance
    drivetrain.getHyperdrive().performInitialCalculations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //below method is chainable. See the class and method reference in the manual
    //for methods to modify the trajectory, such as invertDirection() or invertTurn().
    TankTrajectory newTrajectory = drivetrain.getHyperdrive()
                                             .calculateNextMovements()
                                             .getTankTrajectory(Constants.ROBOT_WHEELBASE_WIDTH)
                                             .convertTime(Units.TIME.MINUTES);
    
    double
      newLeftOutput = newTrajectory.getLeftPercentOutput(drivetrain.getLeftVelocityMotorUnits()),
      newRightOutput = newTrajectory.getRightPercentOutput(drivetrain.getRightVelocityMotorUnits());

    //this line will change depending on how your drivetrain is written, but you get the idea.
    drivetrain.setPercentOutputs(newLeftOutput, newRightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //notifies Hyperdrive that the path is finished.
    drivetrain.getHyperdrive().finishPath();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getHyperdrive().pathFinished();
  }
}
```

## PathVisualizer
PathVisualizer is a graphical application that can be used to view paths that the robot drives. It can communicate directly with the robot to receive and display real-time robot position and path data. It can also be used to load paths from and save paths to the robot. Simply enter the robot's IPv4 address and port, then press "connect".
![logo](https://github.com/BTK203/Hyperdrive/blob/develop/images/pathvisualizer.png?raw=true)

