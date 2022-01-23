// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive;

/**
 * This class stores constants (public static final) that are used by Hyperdrive.
 */
public class HyperdriveConstants {
    //this constructor exists so that no one can create an instance of this class.
    private HyperdriveConstants() {
    }

    //recorder constants
    public static final int PATH_RECORDER_DISTANCE_INTERVAL = 4; //inches

    public static final String 
        PATH_RECORDER_DEFAULT_RECORD_PATH = "/home/lvuser/points.txt",
        PATH_EMULATOR_DEFAULT_RESULTS_PATH = "/home/lvuser/results.txt";

    //emulator constants
    public static final int
        EMULATE_POINT_PASS_LIMIT = 10,
        EMULATE_MAX_HEADING_TO_TURN_DIFFERENCE = 75,
        EMULATE_IMMEDIATE_PATH_SIZE = 2;
    
    public static final double
        EMULATE_NEGLIGIBLE_DEVIANCE = 0.01;

    //PositionTracker constants
    public static final double
        ALMOST_ZERO = 1.2; // This is the value that the absoulte value of the robot's displacement should be under to be considered "zeroed."

    public static final boolean 
        USE_CHORD_BASED_TRACKING = true;

    //PathVisualizer constants
    public static final int
        SOCKET_BUFFER_SIZE = 16000;

    public static final String
        START_SEQUENCE = "[",
        END_SEQUENCE = "]",
        SUBJECT_SEQUENCE = "---",
        SPLIT_SEQUENCE = ":::";

    //default values
    public static final double
        DEFAULT_MIN_SPEED_IPS = 50,
        DEFAULT_MAX_SPEED_IPS = 100,
        DEFAULT_POSITIONAL_CORRECT_INHIBITOR = 0.025,
        DEFAULT_COEFFICIENT_OF_STATIC_FRICTION = 0.5;

    //simulator constants
    public static final double
        INTERNAL_FRICTION_ACCELERATION = 1; // m/s. Based on my experience pushing unpowered robots across rooms and timing their stops
}
