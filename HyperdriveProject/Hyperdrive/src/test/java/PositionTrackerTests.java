import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.PositionTracker;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Test class for the {@link PositionTracker} class. */
public class PositionTrackerTests {
    @Test
    public void testBasicTracking() {
        PositionTracker tracker = new PositionTracker(1);
        
        //first test (straight forward in positive x direction)
        tracker.update(1, 0);

        final Point2D expected1 = new Point2D(1, 0, 0);
        Point2D actual = tracker.getPositionAndHeading();
        
        assertEquals(expected1.getX(), actual.getX(), 0.01);
        assertEquals(expected1.getY(), actual.getY(), 0.01);
        assertEquals(expected1.getHeading(), actual.getHeading(), 0.01);

        //second test (straight forward in positive y direction)
        tracker.update(2, 90);

        final Point2D expected2 = new Point2D(1.63661977, 0.63661977, 90); //remember that the position tracker accounts for continuous changes in heading
        Point2D actual2 = tracker.getPositionAndHeading();

        assertEquals(expected2.getX(), actual2.getX(), 0.01);
        assertEquals(expected2.getY(), actual2.getY(), 0.01);
        assertEquals(expected2.getHeading(), actual2.getHeading(), 0.01);

        //third test (straight forward in negative x direction)
        tracker.update(3, 180);

        final Point2D expected3 = new Point2D(1, 1.27, 180);
        Point2D actual3 = tracker.getPositionAndHeading();

        assertEquals(expected3.getX(), actual3.getX(), 0.01);
        assertEquals(expected3.getY(), actual3.getY(), 0.01);
        assertEquals(expected3.getHeading(), actual3.getHeading(), 0.01);

        //forth test (negative y direction)
        tracker.update(4, 270);

        final Point2D expected4 = new Point2D(0.36, 0.64, 270);
        Point2D actual4 = tracker.getPositionAndHeading();

        assertEquals(expected4.getX(), actual4.getX(), 0.01);
        assertEquals(expected4.getY(), actual4.getY(), 0.01);
        assertEquals(expected4.getHeading(), actual4.getHeading(), 0.01);
    }

    @Test
    public void testMisc() {
        PositionTracker tracker = new PositionTracker(1, 8, 9, 64);

        //assert current position for the sake of the other tests
        final Point2D expected = new Point2D(8, 9, 64);
        Point2D actual = tracker.getPositionAndHeading();

        assertEquals(expected.getX(), actual.getX(), 0.01);
        assertEquals(expected.getY(), actual.getY(), 0.01);
        assertEquals(expected.getHeading(), actual.getHeading(), 0.01);

        //test zeroing without encoders
        tracker.zeroPositionAndHeading(); //equivilent to zeroPositionAndHeading(false);
        final Point2D zero = new Point2D(0, 0, 0);
        Point2D actual2 = tracker.getPositionAndHeading();

        assertEquals(zero.getX(), actual2.getX(), 0.01);
        assertEquals(zero.getY(), actual2.getY(), 0.01);
        assertEquals(zero.getHeading(), actual2.getHeading(), 0.01);

        //zero before encoders zero. First, unzero "encoders"
        tracker.update(1000, 0);

        final Point2D expected3 = new Point2D(2, 3, 4);
        tracker.setPositionAndHeading(expected3);
        tracker.zeroPositionAndHeading(true);
        Point2D actual3 = tracker.getPositionAndHeading();

        assertEquals(expected3.getX(), actual3.getX(), 0.01);
        assertEquals(expected3.getY(), actual3.getY(), 0.01);
        assertEquals(expected3.getHeading(), actual3.getHeading(), 0.01);

        //...now zero "encoders"
        tracker.update(0, 0);
        Point2D actual4 = tracker.getPositionAndHeading();

        assertEquals(zero.getX(), actual4.getX(), 0.01);
        assertEquals(zero.getY(), actual4.getY(), 0.01);
        assertEquals(zero.getHeading(), actual4.getHeading(), 0.01);
    }
}
