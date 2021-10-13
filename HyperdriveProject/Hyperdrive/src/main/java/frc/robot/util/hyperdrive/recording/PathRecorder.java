// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.recording;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.hyperdrive.HyperdriveConstants;
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.Units;

/**
 * A utility for recording paths.
 */
public class PathRecorder {
    private String file;
    private BufferedWriter buffer;
    private FileWriter writer;
    private Point2D lastPoint;
    private long lastFlushTime;
    private boolean initialized;

    private final Units.LENGTH distanceUnits;
    
    /**
     * Creates a new PathRecorder writing to the given file.
     * @param file The absolute file path of the file to record to.
     * @param distanceUnits The units of length to use.
     */
    public PathRecorder(String file, final Units.LENGTH distanceUnits) {
        this.file = file;
        lastPoint = new Point2D(0, 0, 0);
        lastFlushTime = System.currentTimeMillis();
        initialized = false;
        this.distanceUnits = distanceUnits;
    }

    /**
     * Opens the path file and prepares PathRecorder to write to it.
     * This method MUST be called before recording a path or the path will
     * never actually be written.
     */
    public void init(){
        try {
            writer = new FileWriter(file, false);
            buffer = new BufferedWriter(writer);
            initialized = true;
        } catch (IOException ex) {
            //this will happen when simulating the robot on a Windows system (/home/lvuser doesn't exist),
            //so this part will try opening the file in the local directory.
            try {
                //get the last slash in the file path to figure out the local file name.
                DriverStation.reportWarning(
                    "IO Exception on primary file path " + file + ", falling back to local path.\n" +
                    "Exception: " + ex.getMessage(), true);

                int
                    lastFSlash = file.lastIndexOf("/"),
                    lastBSlash = file.lastIndexOf("\\"),
                    slash = (lastFSlash > -1 ? lastFSlash : lastBSlash);
                    
                String fallbackPath = file.substring(slash + 1);

                if(slash < 0) {
                    DriverStation.reportError("Could not resolve fallback path.", true);
                    return;
                }

                writer = new FileWriter(fallbackPath);
                buffer = new BufferedWriter(writer);
                initialized = true;
                file = fallbackPath;
            } catch (IOException ex2) {
                DriverStation.reportError("IO EXCEPTION ON FALLBACK: " + ex2.getMessage(), true);
            }
        }
    }

    /**
     * Closes the path file after recording.
     */
    public void closeFile() {
        if(!initialized) {
            return;
        }

        try {
            flushFile();
            writer.close();
            buffer.close();
            initialized = false;
        } catch(IOException ex) {
            DriverStation.reportError("IO EXCEPTION", true);
        }
    }

    /**
     * Updates the PathRecorder with the current robot position. PathRecorder
     * will decide if the point should be recorded and also if the file should be 
     * updated.
     * @param point The current robot position and heading.
     */
    public void recordPoint(Point2D point) {
        try {
            final double distanceIntervalUnits = HyperdriveUtil.convertDistance(HyperdriveConstants.PATH_RECORDER_DISTANCE_INTERVAL, Units.LENGTH.INCHES, distanceUnits);
            if(point.getDistanceFrom(lastPoint) >= distanceIntervalUnits) {
                buffer.append(point.toString() + "\n");
                lastPoint = point;
            }

            //decide if we should flush
            long currentTime = System.currentTimeMillis();
            if(currentTime - lastFlushTime > 1000) {
                flushFile();
                lastFlushTime = currentTime;
            }
        } catch(IOException ex) {
            DriverStation.reportError("IO EXCEPTION OCCURRED", true);
        }
    }

    /**
     * Forces the PathRecorder to update the path file. 
     * @throws IOException If the file operation fails.
     */
    public void flushFile() throws IOException {
        if(buffer == null) {
            DriverStation.reportError("Recorder does not have a valid file Path! In order for your path to record, ensure that the file path exists!", true);
            return;
        }

        buffer.flush();
    }

    /**
     * Returns the absolute file path of the Path that the PathRecorder is writing to.
     * @return Path to the currently recording file, as a String.
     */
    public String getFilePath() {
        return file;
    }
}
