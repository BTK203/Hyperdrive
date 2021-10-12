// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.enumeration.DriveStyle;
import frc.robot.util.hyperdrive.recording.PathRecorder;

/** 
 * Big brain that does the path trajectory calculations.
 * <br><br>
 * This class is used by the {@link Hyperdrive} class to perform the calculations
 * pertaining to path emulation. That is, this class is what figures out what the robot 
 * needs to do to stay on course.
 */
public class PathEmulator {
    private Path path;
    private IEmulateParams parameters;
    private IController controller;
    private PathRecorder recorder;
    private boolean 
        isForwards,
        pathFinished;

    private int currentPointIndex;
    private final double 
        motorUnitsPerUnit,
        robotWeightNewtons;

    private final DriveStyle driveStyle;
    private final Units.LENGTH lengthUnit;

    /**
     * Creates a new PathEmulator that is pre-loaded with the given {@code Path}
     * and {@code IEmulateParams}. Any call to the {@link #isLoaded()} method will
     * return true unless the {@link #load(Path, IEmulateParams)} method is called with {@code null} objects.
     * @param driveStyle The drivetrain style of the robot.
     * @param motorUnitsPerUnit The number of motor units that equate to one actual unit. 
     * For full explanation, see the constructor for the {@link Hyperdrive} class.
     * @param lengthUnit The unit of distance that will be used.
     * @param robotWeight The weight of the robot in either pounds, newtons, or kilogram-force.
     * @param weightUnit The unit of weight that the robot is measured in.
     * @param path The Path to pre-load
     * @param parameters The IEmulateParams to pre-load
     */
    public PathEmulator(
        final DriveStyle driveStyle,
        final double motorUnitsPerUnit, 
        final Units.LENGTH lengthUnit, 
        final double robotWeight, 
        final Units.FORCE weightUnit, 
        Path path, 
        IEmulateParams parameters
    ) {
        this.driveStyle = driveStyle;
        this.path = path;
        this.parameters = parameters;
        this.motorUnitsPerUnit = motorUnitsPerUnit;
        this.robotWeightNewtons = HyperdriveUtil.convertForce(robotWeight, weightUnit, Units.FORCE.NEWTON);
        this.lengthUnit = lengthUnit;
        this.recorder = new PathRecorder(HyperdriveConstants.PATH_EMULATOR_DEFAULT_RESULTS_PATH, lengthUnit);
        this.isForwards = true;
        this.pathFinished = true; //call load() to correct this
        this.currentPointIndex = 0;

        //create placeholder controller. Actual controller will be created by the performInitialCalculations() method
        //before emulation starts
        createController();
    }

    /**
     * Creates an empty PathEmulator. The object will not do anything or return any
     * non-zero calculation until its path and parameters are set using {@link #load(Path, IEmulateParams)}.
     * Users can check the state of the PathEmulator using the {@link #isLoaded()} method.
     * @param driveStyle The drivetrain style of the robot.
     * @param motorUnitsPerUnit The number of motor units that equate to one actual unit. 
     * @param lengthUnit The unit of distance that should be used. The motorUnitsPerUnit value should convert
     * For full explantion, see the constructor for the {@link Hyperdrive} class.
     * @param robotWeight The weight of the robot in either pounds, newtons, or kilogram-force.
     * @param weightUnit The unit of weight that the robot was measured in.
     * motor units to this unit.
     */
    public PathEmulator(
        final DriveStyle driveStyle,
        final double motorUnitsPerUnit, 
        final Units.LENGTH lengthUnit, 
        final double robotWeight, 
        Units.FORCE weightUnit
    ) {
        this(
            driveStyle, 
            motorUnitsPerUnit, 
            lengthUnit, 
            robotWeight, 
            weightUnit, 
            new Path(new Point2D[] { new Point2D(0, 0, 0) }),
            new PreferenceEmulationParams(lengthUnit)
        );
    }

    /**
     * Loads the PathEmulator with the given {@link Path} and {@link IEmulateParams}. All trajectories
     * calculated after this call will then align the robot to this Path, rather than the one before
     * it. After calling this with non-null objects, all subsequent calls to {@link #isLoaded()} should 
     * return {@code true} until this method is called again with {@code null} objects.
     * @param path The new Path to drive.
     * @param parameters The parameters to use while emulating the path.
     */
    public void load(Path path, IEmulateParams parameters) {
        this.path = path;
        this.parameters = parameters;
    }

    /**
     * Changes the location of the file where results are stored.
     * @param filePath The new file to record path results to.
     */
    public void specifyResultsFile(String filePath) {
        recorder.closeFile();
        recorder = new PathRecorder(filePath, lengthUnit);
    }

    /**
     * Returns the {@link Path} that the robot just took while trying to drive the path supplied to this class.
     * @return Previously driven Path.
     */
    public Path getDrivenPath() {
        return new Path(recorder.getFilePath());
    }

    /**
     * Performs calculations that are required for Hyperdrive to drive a path to the best of its ability.
     * Mainly, this method uses the current robot position to figure out if it should start going forwards
     * or backwards.
     * @param robotPosition The current position of the robot.
     */
    public void performInitialCalculations(Point2D robotPosition) {
        //perform assertions for path following
        boolean
            pathLoaded     = isLoaded(),
            pathValid      = path.isValid(),
            pathLongEnough = path.getPoints().length > 1;

        recorder.init();

        if(pathLoaded && pathValid && pathLongEnough) {
            double headingToNextPoint = robotPosition.getHeadingTo(path.getPoints()[1]);
            double headingDifference = HyperdriveUtil.getAngleToHeading(robotPosition.getHeading(), headingToNextPoint); 
            this.isForwards = Math.abs(headingDifference) < 90;
            
            currentPointIndex = 0;
        } else {
            //at least one of the conditions in the if statement was false. Print out an error message for the ones that were.
            if(!pathLoaded) {
                DriverStation.reportError("No Path is loaded!", true);
            } else if(!pathValid) {
                DriverStation.reportError("The loaded Path is not valid!", true);
            } else if(!pathLongEnough) {
                DriverStation.reportError("The loaded Path is not long enough! Paths must be longer than 1 point!", true);
            }

            pathFinished = true;
        }

        //now that path and parameters are loaded, and the robot is ready to emulate a path, create the controller that it 
        //will be driven with.
        createController();
    }

    /**
     * Frees up resources used by the PathEmulator during path emulation.
     */
    public void end() {
        recorder.closeFile();
    }

    /**
     * Returns the {@link Path} that the robot is currently running.
     * @return The path that the robot is driving.
     */
    public Path getPath() {
        return path;
    }

    /**
     * Returns the index of the current point that the robot is trying to achieve.
     * A value of 0 indicates that the robot is at the beginning of the path.
     * @return Current point index.
     */
    public int getCurrentPoint() {
        return currentPointIndex;
    }

    /**
     * Returns the {@link IEmulateParams} That are currently being used to make the robot drive its path.
     * @return Path emulation parameters.
     */
    public IEmulateParams getParameters() {
        return parameters;
    }

    /**
     * Determines whether or not this PathEmulator has both a path and parameters to do trajectory 
     * calculations with. If this method returns false, any calls to {@link #calculateTrajectory(Point2D)}
     * will return trajectories with 0 speed, 0 displacement, and 0 turn, which will cause the robot
     * to stand still. Should this happen, this PathEmulator's {@link #load(Path, IEmulateParams)}
     * method must be called with non-null objects.
     * @return {@code true} if the PathEmulator is ready to do trajectory calculations, and {@code false} otherwise.
     */
    public boolean isLoaded() {
        return path != null && parameters != null;
    }

    /**
     * Returns whether or not the robot has completed the Path.
     * @return True if the path is complete, false otherwise.
     */
    public boolean isFinished() {
        return pathFinished;
    }

    /**
     * Calculates the trajectory that the robot needs to take in order for it to stay on course with
     * the Path that it is currently running. 
     * @param robotPosition The current position of the robot on the field.
     * @return The {@link Trajectory} that the robot needs to use to stay on course with its Path.
     */
    public Trajectory calculateTrajectory(Point2D robotPosition) {
        Point2D[] points = path.getPoints();
        recorder.recordPoint(robotPosition);

        //if path is too short, then end the path.
        if(points.length <= 1) {
            pathFinished = true;
            return new Trajectory(0, 0, 0, parameters, controller, motorUnitsPerUnit); //no movement trajectory
        }
        
        //resolve the point that the robot is currently at and where we want to aim
        if(currentPointIndex < points.length - 1) {
            double currentDirection = forwardsify(robotPosition.getHeading());
            for(int limit=0; limit<HyperdriveConstants.EMULATE_POINT_PASS_LIMIT; limit++) {
                //get the angle that the robot needs to turn to acheive the point
                double headingToNext = Math.abs(HyperdriveUtil.getAngleToHeading(currentDirection, robotPosition.getHeadingTo(points[currentPointIndex])));

                //get a path that consists of future points
                if(currentPointIndex < points.length - 1 && headingToNext >= 90) {
                    currentPointIndex++;
                } else {
                    break;
                }
            }
        }

        int skipCount = parameters.getPointSkipCount();
        currentPointIndex = (currentPointIndex > points.length - skipCount ? points.length - skipCount : currentPointIndex);
        Point2D currentDestination = points[currentPointIndex + 1];

        //figure out if the robot needs to drive forwards or backwards to acheive the point
        double headingToNextPoint = robotPosition.getHeadingTo(currentDestination);
        double headingDifference = HyperdriveUtil.getAngleToHeading(robotPosition.getHeading(), headingToNextPoint); 
        this.isForwards = Math.abs(headingDifference) < 90;

        //Resolve the path of points that are immediately ahead of the robot. This array will include the robot's location as the first point.
        int immediatePathSize = parameters.getImmediatePathSize();
        Point2D[] nextPoints = getNextNPoints(points, currentPointIndex + skipCount, immediatePathSize);
        Point2D[] immediatePath = new Point2D[nextPoints.length + 1];

        //set first point to robot location, but the heading must be forwards trajectory. Fill in rest of immediate path with normal points
        immediatePath[0] = new Point2D(robotPosition.getX(), robotPosition.getY(), forwardsify(robotPosition.getHeading()));
        for(int i=1; i<immediatePath.length; i++) {
            immediatePath[i] = nextPoints[i - 1];
        }

        if(immediatePath.length < 2) {
            return new Trajectory(0, 0, 0, parameters, controller, motorUnitsPerUnit);
        }

        //draw an "arc" that closely fits the path. The arc will be used to calculate the robot velocity and turn magnitude.
        double immediateDistance = getDistanceOfPath(immediatePath); //unit: in
        double immediateTurn = getTurnOfPath(immediatePath); //unit: degrees
        double headingChange = HyperdriveUtil.getAngleToHeading(immediatePath[1].getHeading(), immediatePath[immediatePath.length - 1].getHeading());

        SmartDashboard.putNumber("immediate turn", immediateTurn);
        SmartDashboard.putNumber("immediate dist", immediateDistance);
        SmartDashboard.putNumber("heading change", headingChange);
        SmartDashboard.putNumber("point", currentPointIndex);

        //figure out if the robot is about switch directions (forward to backward or vice versa). If so, the robot will want to make a large turn. So we zero it.
        double turnToHeadingDifference = Math.abs(HyperdriveUtil.getAngleToHeading(headingChange, immediateTurn));
        boolean shouldZeroTurn = turnToHeadingDifference > HyperdriveConstants.EMULATE_MAX_HEADING_TO_TURN_DIFFERENCE;    

        //add positional correction to heading by making the robot aim for 2 points ahead of us
        Point2D targetPoint = points[currentPointIndex + 2];
        if(robotPosition.getDistanceFrom(targetPoint) > parameters.getPositionalCorrectionDistance()) {
            double positionalCorrection = HyperdriveUtil.getAngleToHeading(forwardsify(robotPosition.getHeading()), robotPosition.getHeadingTo(targetPoint));
            positionalCorrection *= robotPosition.getDistanceFrom(targetPoint) * parameters.getPositionalCorrectionInhibitor();
            immediateTurn += positionalCorrection;
        }

        immediateTurn *= parameters.getOverturn();

        immediateTurn = Math.toRadians(immediateTurn); //The Trajectory class requires values in radians.
        double radius = immediateDistance / immediateTurn;
        double baseSpeed = calculateBestTangentialSpeed(radius);
        double velocity = (isForwards ? baseSpeed : -1 * baseSpeed);
        
        if(shouldZeroTurn) {
            immediateTurn = 0;
        }

        pathFinished = currentPointIndex >= path.getPoints().length - parameters.getPointSkipCount() - 2; //path will be finished when the immediate path can only be two points long.
        return new Trajectory(velocity, immediateDistance, immediateTurn, parameters, controller, motorUnitsPerUnit);
    }

    /**
     * Returns an "n" long array of points, starting at start.
     * @param baseArray The array to create a sub-array from.
     * @param start     The index to start the sub-array from.
     * @param n         The length of the sub-array.
     * @return An "n" long array of Point2D objects. May be shorter if forbidden indices exist (start + n > length).
     */
    private Point2D[] getNextNPoints(Point2D[] baseArray, int start, int n) {
        int end = start + n;
        end = (end > baseArray.length ? baseArray.length : end);

        Point2D[] points = new Point2D[end - start];
        for(int i=start; i<end; i++) {
            points[i - start] = baseArray[i];
        }

        return points;
    }

    /**
     * Returns the sum of the distance between all points of a path.
     * @param path An array of points representing the path.
     * @return The approximate distance of the path.
     */
    private double getDistanceOfPath(Point2D[] path) {
        double distance = 0;
        for(int i=0; i<path.length - 1; i++) {
            distance += path[i].getDistanceFrom(path[i + 1]);
        }

        return distance;
    }

    /**
     * Returns the average turn of a path.
     * @param path An array of points representing the path.
     * @return The average turn of the path in degrees.
     */
    private double getTurnOfPath(Point2D[] path) {
        double turn = 0;
        double lastHeading = path[0].getHeading();
        for(int i=1; i<path.length; i++) {
            double headingToPoint = path[i - 1].getHeadingTo(path[i]);
            double correctionToPoint = HyperdriveUtil.getAngleToHeading(lastHeading, headingToPoint);

            turn += correctionToPoint;
            lastHeading = headingToPoint;
        }

        return turn;
    }
  
    /**
     * Returns an angle corresponding to the direction that the robot is travelling in
     * @param angle Original angle.
     */
    private double forwardsify(double angle) {
        return (isForwards ? angle : (angle + 180) % 360);
    }

    /**
     * Calculates the best speed that the robot should drive through an arc at.
     * @param turnRadius The radius of the turn that the robot will take in inches.
     * @return The best speed for the turn in in/sec
     */
    private double calculateBestTangentialSpeed(double turnRadius) {
        double maxSpeed = parameters.getMaximumSpeed();
        double minSpeed = parameters.getMinimumSpeed();
        if(Double.isNaN(turnRadius)) {
            return minSpeed;
        }

        //gather needed variables (coefficient of friction, normal force, and mass) and convert to SI units.
        double coefficientOfFriction = parameters.getCoefficientOfStaticFriction(); //No Unit.
        double normalForce = robotWeightNewtons; //unit: N. There is no extra downwards force on the robot so Fn == Fg
        double robotMass   = HyperdriveUtil.massKGFromWeight(robotWeightNewtons, Units.FORCE.NEWTON); //unit: kg
        double radius      = Math.abs(HyperdriveUtil.convertDistance(turnRadius, lengthUnit, Units.LENGTH.METERS)); //unit: m. We can absolute value it because we dont care about the direction of the arc.

        //formula: v = sqrt( (r * CoF * Fn) / m )
        double bestSpeed = Math.sqrt( ( radius * coefficientOfFriction * normalForce ) / robotMass ); //unit: m/s

        //convert best speed to in/s
        bestSpeed = HyperdriveUtil.convertDistance(bestSpeed, Units.LENGTH.METERS, lengthUnit); //unit: in/s
        bestSpeed = (bestSpeed > maxSpeed ? maxSpeed : (bestSpeed < minSpeed ? minSpeed : bestSpeed));

        return bestSpeed;
    }

    /**
     * Creates and sets a new {@link IController}, using the current values of parameters and driveStyle.
     */
    private void createController() {
        switch(driveStyle) {
            case TANK: {
                controller = new TankController(parameters);
            }
            break;

            default: {
                String msg = "DriveStyle " + driveStyle + " not supported!";
                DriverStation.reportError(msg, true);
                throw new RuntimeException(msg);
            }
        }
    }
}