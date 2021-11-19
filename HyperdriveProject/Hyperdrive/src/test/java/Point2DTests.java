import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.util.hyperdrive.util.Point2D;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Test class for the Point2D class. */
public class Point2DTests {
    @Test
    public void testGetDistanceFrom() {
        //test 1 (0 distance)
        Point2D
            point1 = new Point2D(0, 0, 0),
            point2 = new Point2D(0, 0, 0);

        assertEquals(0, point1.getDistanceFrom(point2), 0.01);
        assertEquals(0, point2.getDistanceFrom(point1), 0.01);
        assertEquals(0, point1.getDistanceFrom(point1), 0.01); //point's distance from itself

        //test 2 (x axis distance)
        double distance = Math.random() * 100;
        point1 = new Point2D(0, 0, 0);
        point2 = new Point2D(distance, 0, 0);
        assertEquals(distance, point1.getDistanceFrom(point2), 0.01);
        assertEquals(distance, point2.getDistanceFrom(point1), 0.01);

        //test 3 (y axis distance)
        double distance2 = Math.random() * 100;
        point1 = new Point2D(0, 0, 0);
        point2 = new Point2D(0, distance2, 0);
        assertEquals(distance2, point1.getDistanceFrom(point2), 0.01);
        assertEquals(distance2, point2.getDistanceFrom(point1), 0.01);

        //test 3 (both axes)
        point1 = new Point2D(6, 3, 0);
        point2 = new Point2D(-8, 1, 0);
        assertEquals(14.1421, point1.getDistanceFrom(point2), 0.01);
        assertEquals(14.1421, point2.getDistanceFrom(point1), 0.01);
    }

    @Test
    public void testGetHeadingTo() {
        //test 1 (0 heading)
        Point2D
            point1 = new Point2D(0, 0, 0),
            point2 = new Point2D(1, 0, 0);

        assertEquals(0, point1.getHeadingTo(point2), 0.01);
        assertEquals(180, point2.getHeadingTo(point1), 0.01);
        
        //test 2 (simple 90 degrees)
        point1 = new Point2D(1, 1, 0);
        point2 = new Point2D(1, 2, 90); //the 90 degree heading is a demonstration that point heading is not considered by getHeadingTo()
        assertEquals(90, point1.getHeadingTo(point2), 0.01);
        assertEquals(-90, point2.getHeadingTo(point1), 0.01);

        //test 3 (complex weird angle)
        point1 = new Point2D(-4, -6, 8);
        point2 = new Point2D(2, -9, 0);
        assertEquals(-26.5651, point1.getHeadingTo(point2), 0.01);
        assertEquals(153.4349, point2.getHeadingTo(point1), 0.01);
    }

    @Test
    public void testMisc() {
        String[] points = {
            "1.0,2.0,3.0",
            "1.0,6.0,5.0",
            "100.0,70.0,-89.0",
            "54.0,83.0,0.0",
            "-23.0,-76.0,-3.0"
        };

        double[] results = {
            1,2,3,
            1,6,5,
            100,70,-89,
            54,83,0,
            -23,-76,-3
        };

        //assert here to make sure that the rest of the tests will perform as predicted
        assertEquals(points.length, results.length / 3);

        for(int i=0; i<points.length; i++) {
            //test creating the point from the string
            Point2D test = Point2D.fromString(points[i]);

            double
                expectedX = results[(3 * i) + 0],
                expectedY = results[(3 * i) + 1],
                expectedH = results[(3 * i) + 2];

            assertEquals(expectedX, test.getX(), 0.01);
            assertEquals(expectedY, test.getY(), 0.01);
            assertEquals(expectedH, test.getHeading(), 0.01);

            //test converting to string
            String testString = test.toString();
            System.out.println("testString: " + testString);
            assertEquals(points[i], testString);
        }
    }
}
