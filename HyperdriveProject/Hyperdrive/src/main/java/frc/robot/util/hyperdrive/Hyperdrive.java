// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.emulation.PathEmulator;
import frc.robot.util.hyperdrive.emulation.Trajectory;
import frc.robot.util.hyperdrive.pathvisualizer.PVHost;
import frc.robot.util.hyperdrive.recording.PathRecorder;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.PositionTracker;
import frc.robot.util.hyperdrive.util.Units;

/** 
 * A tool for autonomously driving the robot around. This tool will handle communication with the PathVisualizer app 
 * and be able to record paths and emulate them with a few simple lines of code. 
 * To start, instantiate an instance of this class into your drivetrain subsystem, and then call the 
 * {@link #update(double distanceTravelled, double direction)} method or the 
 * {@link #update(double leftDistance, double rightDistance, double heading)} in 
 * the drivetrain's {@code periodic()} method. 
 */
public class Hyperdrive {
    private PVHost          pvHost;
    private PositionTracker tracker;
    private PathRecorder    recorder;
    private PathEmulator    emulator;
    private boolean         currentlyRecording;
    private Units.DISTANCE  distanceUnits;

    /**
     * Creates a new Hyperdrive. 
     * @param distanceUnit The units of distance that the robot will use.
     * @param motorUnitsPerUnit The number of motor position units (ticks, rotations, etc) that are in one
     * length measurement unit (inches, feet, meters, etc). To get this value, drive the robot a set distance
     * (more than 10 feet for most accuracy) and then record the raw displacement of the motor encoders.
     * This value should then be equivilent to that displacement divided by the set distance. This value
     * does not absolutely need to be accurate (the robot will emulate any path as long as it was recorded 
     * with the same value), but is required if the position tracker is to report in the correct units.
     * This value defaults to 0.472, which is the approximate rotations per inch value of NEO motors driving 
     * wheels with pneumatic tires through a toughbox mini gearbox.
     * @param robotWeight The weight of the robot.
     * @param weightUnit The units of weight that the robot is measured in.
     * @param pvPort The port to use for communicating with PathVisualizer. Defaults to 3695.
     */
    public Hyperdrive(Units.DISTANCE distanceUnit, final double motorUnitsPerUnit, final double robotWeight, Units.FORCE weightUnit, int pvPort) {
        this.distanceUnits = distanceUnit;
        this.pvHost = new PVHost(pvPort);
        this.tracker = new PositionTracker(motorUnitsPerUnit);
        this.recorder = new PathRecorder(HyperdriveConstants.PATH_RECORDER_DEFAULT_RECORD_PATH);
        this.emulator = new PathEmulator(motorUnitsPerUnit, robotWeight, weightUnit, distanceUnit);
    }

    /**
     * Creates a new Hyperdrive, using the default PathVisualizer port, 3695.
     * @param distanceUnits The units to measure length in.
     * @param motorUnitsPerUnit The number of motor position units (ticks, rotations, etc) that are in one 
     * length measurement unit (inches, feet, meters, etc). See javadoc for
     * {@link #Hyperdrive(frc.robot.util.hyperdrive.util.Units.DISTANCE, double, double, frc.robot.util.hyperdrive.util.Units.FORCE, int)}
     * for full explanation.
     * @param robotWeight The weight of the robot.
     * @param weightUnit The unit of weight that the robot is measured in.
     */
    public Hyperdrive(Units.DISTANCE distanceUnits, final double motorUnitsPerUnit, final double robotWeight, Units.FORCE weightUnit) {
        this(distanceUnits, motorUnitsPerUnit, robotWeight, weightUnit, 3695);
    }

    /**
     * Creates a new Hyperdrive, using the default robot weight, 125 pounds, and PathVisualizer port, 3695.
     * @param distanceUnits The units to measure length in.
     * @param motorUnitsPerUnit The number of motor position units (ticks, rotations, etc) that are in one 
     * length measurement unit (inches, feet, meters, etc). See javadoc for
     * {@link #Hyperdrive(frc.robot.util.hyperdrive.util.Units.DISTANCE, double, double, frc.robot.util.hyperdrive.util.Units.FORCE, int)}
     * for full explanation.
     */
    public Hyperdrive(Units.DISTANCE distanceUnits, final double motorUnitsPerUnit) {
        this(distanceUnits, motorUnitsPerUnit, 125, Units.FORCE.POUND);
    }

    /**
     * Creates a new Hyperdrive, using the default motor units per unit value, 0.472, the default robot weight, 125 pounds,
     * and the default PathVisualizer port, 3695.
     * @param distanceUnits The units of distance to use.
     */
    public Hyperdrive(Units.DISTANCE distanceUnits) {
        this(distanceUnits, 0.472, 125, Units.FORCE.POUND, 3695);
    }

    /**
     * Creates a new Hyperdrive, using the default distance unit, inches, the default motor units per unit value, 0.472, the 
     * default robot weight, 125 pounds, and the default PathVisualizer port, 3695.
     */
    public Hyperdrive() {
        this(Units.DISTANCE.INCHES);
    }

    /**
     * Updates the Hyperdrive. The Hyperdrive will use the distance that the
     * robot has travelled and the direction that it has travelled in and 
     * calculate the robot's position on the field. That information can be 
     * viewed on SmartDashboard or Shuffleboard as "Current Robot Position",
     * and can be accessed in code with {@link #getRobotPositionAndHeading()}.
     * <br><br>
     * Teams using swerve drive may find that this particular overload of
     * the {@code update()} method is easier to use, as the other was written 
     * more for tank teams.
     * <br><br>
     * It is recommended that this method be called in one of the robot's
     * {@code periodic()} methods. It is possible to call this method in a {@code while}
     * loop in a separate Thread, but the programmer(s) should be careful of how 
     * often they reference encoder values from their motor controllers, as this 
     * can overload the CAN network and cause undesired motor behavior.
     * 
     * @param distanceTravelled The total displacment of the robot since the last zero, in encoder counts.
     * @param direction The direction that the robot is currently travelling in, in degrees.
     */
    public void update(double distanceTravelled, double direction) {
        tracker.update(distanceTravelled, direction);
        Point2D position = getRobotPositionAndHeading();
        pvHost.update(position);
        SmartDashboard.putString("Current Robot Position", position.toString());

        if(currentlyRecording) {
            recorder.recordPoint(position);
        }
    }

    /**
     * Updates the Hyperdrive. The Hyperdrive will use the distance that the
     * robot has travelled and the direction that it has travelled in and 
     * calculate the robot's position on the field. That information can be 
     * viewed on SmartDashboard or Shuffleboard as "Current Robot Position",
     * or can be accessed in code with {@link #getRobotPositionAndHeading()}
     * <br><br> 
     * Tank teams will likely find that this overload of the {@code update()} method
     * is easier to use than the other one, as they can simply pass through the positions
     * of their drive motors.
     * <br><br>
     * It is recommended that this method be called in one of the robot's
     * {@code periodic()} methods. It is possible to call this method in a {@code while}
     * loop in a separate Thread, but the programmer(s) should be careful of how 
     * often they reference encoder values from their motor controllers, as this 
     * can overload the CAN network and cause undesired motor behavior.
     * 
     * @param leftDistance The current position of the left motor encoder.
     * @param rightDistance The current position of the right motor encoder.
     * @param heading The direction the robot is facing, in degrees.
     */
    public void update(double leftDistance, double rightDistance, double heading) {
        update((leftDistance + rightDistance ) / 2, heading);
    }

    /**
     * Returns the current position and heading of the robot in field space. 
     * @return Current robot position and heading.
     */
    public Point2D getRobotPositionAndHeading() {
        return tracker.getPositionAndHeading();
    }

    /**
     * Sets the position and heading of the robot to 0. This method should be called
     * when the robot is standing still to ensure accurate and reliable results.
     * This overload also provides an option to wait for the drive encoders to be 
     * 0 before actually resetting the position, which helps to prevent large 
     * position jumps that may ruin any position-related zero that was recently set. 
     * See the javadoc for the {@link PositionTracker #zeroPositionAndHeading(boolean)} 
     * for full explanation.
     * @param waitForEncoders True if the position tracker should wait for the motor 
     * encoders to zero before resetting the position. This means that the position
     * tracker will not zero until either the {@link #update(double, double)} method 
     * is called with a distanceTravelled value of 0, or the {@link #update(double, double, double)}
     * method is called with leftDistance and rightDistance values that average out
     * to a value that is less than 1. 
     * NOTE that the robot MUST not be moving if this option is set to {@code true}.
     * If the robot is moving and this option is set to {@code true}, the position 
     * tracker could potentially not zero now, and instead zero at an unwanted time.
     */
    public void zeroPositionAndHeading(boolean waitForEncoders) {
        tracker.zeroPositionAndHeading(waitForEncoders);
    }

    /**
     * Sets the position and heading of the robot to 0. This is equivilent to calling
     * {@code zeroPositionAndHeading(false)}. 
     */
    public void zeroPositionAndHeading() {
        tracker.zeroPositionAndHeading();
    }

    /**
     * Sets the position and heading of the robot. This method is useful as it can
     * be called at any time during the robot's operation, even while it is moving.
     * You can use this method to set the robot's position relative to an object, 
     * or use it to set and maintain the robot's true position on the Field. 
     * @param newPositionAndHeading The new Position and Heading of the robot.
     */
    public void setPositionAndHeading(Point2D newPositionAndHeading) {
        tracker.setPositionAndHeading(newPositionAndHeading);
    }

    /**
     * Returns the units of distance that the Hyperdrive has been configured to use.
     * @return The Hyperdrive's units of distance.
     */
    public Units.DISTANCE getDistanceUnits() {
        return distanceUnits;
    }

    /**
     * Sends a {@link Path} to the PathVisualizer client, if there is one. If a PathVisualizer
     * client is connected to the robot, and the "Live" option is enabled, the path being sent
     * with this method will appear on the screen, under the name that was also given to this
     * method.
     * @param path The path to send.
     * @param name The name of the path. Will appear on the manifest as that name.
     */
    public void sendPath(Path path, String name) {
        pvHost.sendPath(path, name);
    }

    /**
     * Decalares, initalizes, and starts the PathRecorder to record a driven Path.
     * After this method is called, the Recorder will automatically record the robot's movement
     * until the {@link #stopRecorder()} method is called.
     * @param file The file to record to.
     */
    public void initializeRecorder(String file) {
        recorder = new PathRecorder(file);
        recorder.init();
        currentlyRecording = true;   
    }

    /**
     * Declares, initializes, and starts the PathRecorder to record a driven Path.
     * The Path will be stored in the default record location: {@link HyperdriveConstants #PATH_RECORDER_DEFAULT_RECORD_PATH}.
     * After this method is called, the Recorder will automatically record the robot's movement
     * until the {@link #stopRecorder()} method is called.
     */
    public void initializeRecorder() {
        initializeRecorder(HyperdriveConstants.PATH_RECORDER_DEFAULT_RECORD_PATH);
    }

    /**
     * Stops the Path Recorder if it is currently active. The path being recorded will be saved
     * to file and the Recorder will no longer record a Path automatically. The recorded Path
     * can be accessed using {@link #getRecordedPath()} until the recorder is started again.
     * This method will also send the Path that was just recorded to PathVisualizer for viewing.
     * If PathVisualizer is connected, and the "Live" option is enabled, then the Path will appear
     * on the screen.
     */
    public void stopRecorder() {
        currentlyRecording = false;
        recorder.closeFile();
        pvHost.sendPath(getRecordedPath(), "Recorded Path");
    }

    /**
     * Returns the {@code Path} that the Path Recorder has just recorded. If no such path exists, the Path's
     * {@code isValid()} method will return false.
     * @return The previously recorded Path.
     */
    public Path getRecordedPath() {
        return new Path(recorder.getFilePath());
    }

    /**
     * Loads a {@link Path} for Hyperdrive to emulate.
     * @param path The path that the robot should drive.
     * @param parameters The parameters to use while driving this path.
     */
    public void loadPath(Path path, IEmulateParams parameters) {
        emulator.load(path, parameters);
    }

    /**
     * Loads a {@link Path} for Hyperdrive to emulate from the given file path.
     * @param filePath The location of the path file which the robot will drive.
     * @param parameters The parameters to use when driving this path.
     */
    public void loadPath(String filePath, IEmulateParams parameters) {
        loadPath(new Path(filePath), parameters);
    }

    /**
     * Loads a {@link Path} for Hyperdrive to emulate. Sets emulation parameters to
     * the defaults.
     * @param path The path for the robot to drive.
     */
    public void loadPath(Path path) {
        emulator.load(path);
    }

    /**
     * Loads a {@link Path} for Hyperdrive to emulate. Sets emualtion parameters to 
     * the defaults.
     * @param filePath The location of the path which the robot will drive.
     */
    public void loadPath(String filePath) {
        emulator.load(new Path(filePath));
    }

    /**
     * Forces Hyperdrive to record the results of the driven path to the file location
     * provided.
     * @param filePath The location of the new results file.
     */
    public void specifyResultsFile(String filePath) {
        emulator.specifyResultsFile(filePath);
    }

    /**
     * Makes the emulator perform some calculations that must take place in order for the
     * path to be driven to the best of Hyperdrive's ability. This method should be called
     * in the {@code initalize()} method of your autonomous drive command.
     */
    public void performInitialCalculations() {
        emulator.performInitialCalculations(tracker.getPositionAndHeading());
    }

    /**
     * Calculates the target velocity of the robot, and the magnitude of any turn that it needs to
     * take, all based on the current position of the robot. From here, the teams with tank-style 
     * robots can call {@code getTankTrajectory()} to calculate the target velocities for the left 
     * and right wheels of their robot. Then, those velocities must be set as setpoints for their 
     * motor velocity PIDs.
     * <br><br>
     * NOTE, however, that depending on the motor controller that is being used, the velocities may
     * need to be converted to the proper time units accepted by the motor controller being used. 
     * If left as-is, the velocities will be returned as motor units per second. For teams running 
     * TalonFX or TalonSRX drivetrains, those values will need to be converted to motor units per 100 ms.
     * This can be achieved by calling {@code convertTime(Units.TIME.DECASECONDS)} on the new TankTrajectory. 
     * Teams running SparkMAX drivetrains will need to convert their velocitiesto motor units per minute, 
     * which can be acheived with {@code convertTime(Units.TIME.MINUTES)}.
     * @return A Trajectory describing the robot's future course, and how to achieve it.
     */
    public Trajectory calculateNextMovements() {
        return emulator.calculateTrajectory(tracker.getPositionAndHeading());
    }

    /**
     * Returns the number of points that are present in the {@link Path} that the robot is current emulating.
     * @return The number of points in the currently loaded Path.
     */
    public int getTotalPoints() {
        return emulator.getPath().getPoints().length;
    }

    /**
     * Returns the index of the point in the {@link Path} that the robot is trying to emulate, that the robot
     * is currently working to achieve.
     * @return Current point index.
     */
    public int getCurrentPoint() {
        return emulator.getCurrentPoint();
    }

    /**
     * Returns true if Hyperdrive is done driving the robot through a Path, and false otherwise.
     * @return Whether or not the loaded path has been driven.
     */
    public boolean pathFinished() {
        return emulator.isFinished();
    }

    /**
     * Frees up resources used while emulating the Path. Also sends the robot's actual Path to PathVisualier
     * for viewing. If PathVisualizer is connected, and the "Live" option is enabled, then the Path will
     * appear on the screen.
     */
    public void end() {
        emulator.end();
        pvHost.sendPath(emulator.getDrivenPath(), "Driven Path");
    }
}
