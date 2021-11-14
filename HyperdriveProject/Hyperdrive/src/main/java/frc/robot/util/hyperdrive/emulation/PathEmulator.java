// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
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
    private double[] velocityMap; //array containing maximum velocities at all points in path
    private double[] devianceMap; //array containing deviances captured when the robot passes the target point
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
     * Performs calculations that are required for Hyperdrive to drive a path.
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
            double headingDifference = HyperdriveUtil.getAngleBetweenHeadings(robotPosition.getHeading(), headingToNextPoint); 
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

        //allocate array for deviance tracking
        devianceMap = new double[path.getPoints().length];

        //create velocity map
        this.velocityMap = calculateVelocityMap();
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
                double headingToNext = Math.abs(HyperdriveUtil.getAngleBetweenHeadings(currentDirection, robotPosition.getHeadingTo(points[currentPointIndex])));

                //get a path that consists of future points
                if(currentPointIndex < points.length - 1 && headingToNext >= 90) {
                    //calculate deviance from current point, aligned to the axis that the point's heading is aligned with.
                    //TODO: IMPLEMENT

                    currentPointIndex++;
                } else {
                    break;
                }
            }
        }

        currentPointIndex = (currentPointIndex > points.length - 2 ? points.length - 2 : currentPointIndex);
        Point2D currentDestination = points[currentPointIndex + 1];

        this.isForwards = calculateIsForwards(robotPosition, currentDestination);        

        //Resolve the path of points that are immediately ahead of the robot. This array will include the robot's location as the first point.
        Point2D[] nextPoints = getNextNPoints(points, currentPointIndex + 1, HyperdriveConstants.EMULATE_IMMEDIATE_PATH_SIZE);
        Point2D[] immediatePath = new Point2D[nextPoints.length + 1];

        //set first point to robot location, but the heading must be forwards trajectory. Fill in rest of immediate path with normal points
        immediatePath[0] = new Point2D(robotPosition.getX(), robotPosition.getY(), forwardsify(robotPosition.getHeading()));
        for(int i=1; i<immediatePath.length; i++) {
            immediatePath[i] = nextPoints[i - 1];
        }

        //dont try to drive short paths
        if(immediatePath.length < 2) {
            return new Trajectory(0, 0, 0, parameters, controller, motorUnitsPerUnit);
        }

        //once we have our immediate path, calculate a trajectory for it
        return calculateTrajectoryForImmediatePath(immediatePath, robotPosition, velocityMap[currentPointIndex]);
    }

    /**
     * Calculates the unsmoothed velocity map for a path.
     * @param points The points of the path to process.
     * @return The rough velocity map for the given points.
     */
    public double[] calculateRoughVelocityMap(Point2D[] points) {
        int mapPopulatedSize = points.length - HyperdriveConstants.EMULATE_IMMEDIATE_PATH_SIZE; //the number of points in the speedmap that are actually defined
        double[] roughMap = new double[points.length];

        for(int i=0; i<mapPopulatedSize; i++) {
            Point2D currentPoint = points[i];
            Point2D[] immediatePath = getNextNPoints(points, i, HyperdriveConstants.EMULATE_IMMEDIATE_PATH_SIZE);

            //figure out if the robot would be driving backwards at this point
            boolean wouldBeForwards = calculateIsForwards(immediatePath[0], immediatePath[immediatePath.length - 1]);

            //forwards-ify first point of immediate path. This is required by calculateTrajectoryForImmediatePath()
            Point2D
                oldPoint = immediatePath[0],
                newPoint = new Point2D(oldPoint.getX(), oldPoint.getY(), forwardsify(oldPoint.getHeading(), wouldBeForwards));

            immediatePath[0] = newPoint;

            //calculate trajectory for the path (theoretically) ahead. Use Double.MAX_VALUE because there is no cap on the speed
            //that calculateTrajectoryForImmediatePath should calculate.
            Trajectory goalTrajectory = calculateTrajectoryForImmediatePath(immediatePath, currentPoint, Double.MAX_VALUE, wouldBeForwards);
            double velocity = goalTrajectory.getVelocity();
            roughMap[i] = velocity;
        }

        return roughMap;
    }

    /**
     * Calculates a speed map based on the currently loaded path and parameters.
     * The speedmap array will then be populated.
     */
    public double[] calculateVelocityMap() {
        final double maxAccel = parameters.getPIDFAConfig().getAccel();
        Point2D[] points = path.getPoints();
        double[] velocityMap = calculateRoughVelocityMap(points);

        HyperdriveUtil.saveValuesToFile(velocityMap, "rough.txt");

        //numbers for controlling the smoothing algorithm's progress
        int
            increment     = 1, //should be either 1 or -1 
            greatestPoint = 0, //furthest point algorithm has reached
            currentPoint  = 0; //current point algorithm is processing

        while(currentPoint < velocityMap.length - 1) {
            //check to see if the beginning of the array has been reached while moving backwards
            if(increment < 0 && currentPoint <= 0) {
                currentPoint = greatestPoint;
                increment = 1;
            }

            //update greatest point if necessary
            if(currentPoint > greatestPoint) {
                greatestPoint = currentPoint;
            }

            double
                currentVelocity       = velocityMap[currentPoint],
                nextVelocity          = velocityMap[currentPoint + increment],
                distanceBetweenPoints = points[currentPoint].getDistanceFrom(points[currentPoint + increment]);
            
            //acceleration currently needed to get between points and acheive target velocities
            double acceleration = (Math.pow(nextVelocity, 2) - Math.pow(currentVelocity, 2)) / (2 * distanceBetweenPoints);

            //the algorithm for actually smoothing the points only works for positive values, so check signs here to perform operations later
            boolean
                currentPositive = currentVelocity >= 0,
                nextPositive    = nextVelocity >= 0;
            
            //invert negative values so that they become positive
            currentVelocity = conditionalInvert(currentVelocity, !currentPositive); //inverts if negative (!positive)
            nextVelocity    = conditionalInvert(nextVelocity, !nextPositive);

            // if the acceleration is too steep, this is the part that smooths it out.
            if(Math.abs(acceleration) > maxAccel) {
                //if the values are on both sides of zero...
                if(currentPositive != nextPositive) {
                    //compute maximum acceptable velocity from stopped. The distanceBetweenPoints / 2 garantees that
                    //the acceleration will not cause an infinite loop
                    double acceptableVelocity = computeAcceptableVelocity(0, distanceBetweenPoints / 2, maxAccel);
                    
                    //create a new current and next velocity. These are capped to the "current" values.
                    double
                        newCurrentVelocity = (acceptableVelocity > Math.abs(currentVelocity) ? currentVelocity : acceptableVelocity),
                        newNextVelocity    = (acceptableVelocity > Math.abs(nextVelocity) ? nextVelocity : acceptableVelocity);
                    
                    //invert the velocities if necessary
                    newCurrentVelocity = conditionalInvert(newCurrentVelocity, !currentPositive);
                    newNextVelocity    = conditionalInvert(newNextVelocity, !nextPositive);

                    //set the velocities
                    velocityMap[currentPoint]     = newCurrentVelocity;
                    velocityMap[currentPoint + increment] = newNextVelocity;

                    //start going backwards because previous velocities may need to be adjusted. Then go back to top of loop.
                    increment = -1;
                    continue;
                }

                //otherwise, value smoothing goes as normal.
                double 
                    signedMaxAccel     = conditionalInvert(maxAccel, acceleration < 0),
                    acceptableVelocity = computeAcceptableVelocity(currentVelocity, distanceBetweenPoints, signedMaxAccel),
                    deltaV             = acceptableVelocity - currentVelocity;

                //smooth value. Lower values are prioritized by the algorithm
                if(deltaV > 0) {
                    //smooth relative to current velocity if "speeding up"
                    velocityMap[currentPoint + increment] = conditionalInvert(acceptableVelocity, !nextPositive);
                } else {
                    //prioritize the lower speed if "slowing down." This means that previous speeds will need to be re-calculated.
                    currentPoint = currentPoint + (2 * increment); //will be decremented at end of loop, making 1 * increment
                    increment = -1;
                }
            } else {
                //if going backwards, we have reached acceptable velocities and can start going forwards again.
                if(increment < 0) {
                    currentPoint = greatestPoint - 1; //will be incremented later, making greatestPoint - 0
                    increment = 1;
                }
            }
            
            currentPoint += increment; //increment / decrement current point
        }

        HyperdriveUtil.saveValuesToFile(velocityMap, "smoothed.txt"); //TODO: DELETE

        return velocityMap;
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
     * @param path An array of points representing the path. The first point should be forwards-ified
     * using {@link #forwardsify(double)}.
     * @return The average turn of the path in degrees.
     */
    private double getTurnOfPath(Point2D[] path) {
        double turn = 0;
        double lastHeading = path[0].getHeading();
        for(int i=1; i<path.length; i++) {
            double headingToPoint = path[i - 1].getHeadingTo(path[i]);
            double correctionToPoint = HyperdriveUtil.getAngleBetweenHeadings(lastHeading, headingToPoint);

            turn += correctionToPoint;
            lastHeading = headingToPoint;
        }

        return turn;
    }
  
    /**
     * Returns an angle corresponding to the direction that the robot is travelling in
     * @param angle Original angle.
     * @param isForwards {@code true} if the robot is currently driving forwards, {@code false} otherwise.
     * @return A new angle with respect to the direction of the robot's movement.
     */
    private double forwardsify(double angle, boolean isForwards) {
        return (isForwards ? angle : (angle + 180) % 360);
    }

    /**
     * Returns an angle corresponding to the direction that the robot is travelling in
     * @param angle Original angle.
     * @return A new angle with respect to the direction of the robot's movement.
     */
    private double forwardsify(double angle) {
        return forwardsify(angle, isForwards);
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

    /**
     * Computes the acceptable next velocity from the current one that will satisfy the maximum acceleration.
     * @param currentVelocity Current velocity of robot
     * @param distance Distance the robot will drive by next velocity
     * @param maxAcceleration Maximum acceleration of robot
     * @return The acceptable velocity of the robot at the next point.
     */
    private double computeAcceptableVelocity(double currentVelocity, double distance, double maxAcceleration) {
        double numberToSqrt = Math.pow(currentVelocity, 2) + (2 * maxAcceleration * distance);
        double acceptableVelocity = 0;

        if(numberToSqrt > 0) {
            acceptableVelocity = Math.sqrt(numberToSqrt);
        } else {
            acceptableVelocity = -1 * Math.sqrt(Math.abs(numberToSqrt));
        }

        return acceptableVelocity;
    }

    /**
     * Inverts a value if {@code shouldInvert} is true.
     * @param value The value to (maybe) invert.
     * @param shouldInvert Whether or not to invert the value. 
     * @return The (maybe) inverted value.
     */
    private double conditionalInvert(double value, boolean shouldInvert) {
        return (shouldInvert ? value * -1 : value);
    }

    /**
     * Calculates the trajectory needed for the robot to drive through a short path immediately ahead of it.
     * @param immediatePath An array of points describing the path immediately ahead of the robot.
     * @param robotPosition The current position of the robot.
     * @param precalculatedVelocity The velocity calculated for the current point before path execution. Will
     * be treated as the maximum allowable robot speed.
     * @param isForwards {@code true} if the robot is driving forwards, {@code false} otherwise.
     * @return A Trajectory object containing information needed to drive the robot through the immediate path.
     */
    private Trajectory calculateTrajectoryForImmediatePath(Point2D[] immediatePath, Point2D robotPosition, double precalculatedVelocity, boolean isForwards) {
        //draw an "arc" that closely fits the path. The arc will be used to calculate the robot velocity and turn magnitude.
        double immediateDistance = getDistanceOfPath(immediatePath); //unit: in
        double immediateTurn = getTurnOfPath(immediatePath); //unit: degrees
        double headingChange = HyperdriveUtil.getAngleBetweenHeadings(immediatePath[1].getHeading(), immediatePath[immediatePath.length - 1].getHeading());

        //figure out if the robot is about switch directions (forward to backward or vice versa). If so, the robot will want to make a large turn. So we zero it.
        double turnToHeadingDifference = Math.abs(HyperdriveUtil.getAngleBetweenHeadings(headingChange, immediateTurn));
        boolean shouldZeroTurn = turnToHeadingDifference > HyperdriveConstants.EMULATE_MAX_HEADING_TO_TURN_DIFFERENCE;    

        //add positional correction to heading by making the robot aim for 2 points ahead of us
        Point2D targetPoint = immediatePath[1];
        double positionalCorrection = HyperdriveUtil.getAngleBetweenHeadings(forwardsify(robotPosition.getHeading(), isForwards), robotPosition.getHeadingTo(targetPoint));
        positionalCorrection *= robotPosition.getDistanceFrom(targetPoint) * parameters.getPositionalCorrectionInhibitor();
        immediateTurn += positionalCorrection;

        immediateTurn = Math.toRadians(immediateTurn); //The Trajectory class requires values in radians.

        double 
            radius = immediateDistance / immediateTurn,
            baseSpeed = calculateBestTangentialSpeed(radius);

        if(baseSpeed > Math.abs(precalculatedVelocity)) {
            baseSpeed = Math.abs(precalculatedVelocity);
        }

        if(baseSpeed < parameters.getMinimumSpeed()) {
            baseSpeed = parameters.getMinimumSpeed();
        }
        
        double velocity = (isForwards ? baseSpeed : -1 * baseSpeed);
        
        if(shouldZeroTurn) {
            immediateTurn = 0;
        }

        //TODO: the line below may need to be line - 3, but if it works, then it works I guess
        pathFinished = currentPointIndex >= path.getPoints().length - 2; //path will be finished when the immediate path can only be two points long.
        return new Trajectory(velocity, immediateDistance, immediateTurn, parameters, controller, motorUnitsPerUnit);
    }

    /**
     * Calculates the trajectory needed for the robot to drive through a short path immediately ahead of it.
     * This implementation uses the class's stored value for isForwards.
     * @param immediatePath An array of points describing the path immediately ahead of the robot.
     * @param robotPosition The current position of the robot.
     * @param precalculatedVelocity The velocity calculated for the current point before path execution. Will
     * be treated as the maximum allowable robot speed.
     * @return A Trajectory object containing information needed to drive the robot through the immediate path.
     */
    private Trajectory calculateTrajectoryForImmediatePath(Point2D[] immediatePath, Point2D robotPosition, double precalculatedVelocity) {
        return calculateTrajectoryForImmediatePath(immediatePath, robotPosition, precalculatedVelocity, isForwards);
    }

    /**
     * Determines whether or not the robot was going fowards at a specified position
     * @param position The current position of the robot.
     * @param destination The point that the robot is aiming for
     * @return {@code true} if the robot should be going forwards, {@code false} otherwise.
     */
    private boolean calculateIsForwards(Point2D position, Point2D destination) {
        //figure out if the robot needs to drive forwards or backwards to acheive the point
        double headingToNextPoint = position.getHeadingTo(destination);
        double headingDifference = HyperdriveUtil.getAngleBetweenHeadings(position.getHeading(), headingToNextPoint); 
        return Math.abs(headingDifference) < 90;
    }
}