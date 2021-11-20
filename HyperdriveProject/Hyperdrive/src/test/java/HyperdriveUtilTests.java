// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Point2D;
import frc.robot.util.hyperdrive.util.Units;

/** 
 * Test class for some foundational Hyperdrive structures.
 */
public class HyperdriveUtilTests { //TODO: Write test for HyperdriveUtil.getDeviance()
    @Test
    public void testLengthUnits() {
        assertEquals(1.667, HyperdriveUtil.convertDistance(20, Units.LENGTH.INCHES, Units.LENGTH.FEET), 0.01); //20 inches = 1.666667 ft
        assertEquals(86.4, HyperdriveUtil.convertDistance(7.2, Units.LENGTH.FEET, Units.LENGTH.INCHES), 0.01);
        assertEquals(2.133, HyperdriveUtil.convertDistance(6.4, Units.LENGTH.FEET, Units.LENGTH.YARDS), 0.01);
        assertEquals(1.05, HyperdriveUtil.convertDistance(0.35, Units.LENGTH.YARDS, Units.LENGTH.FEET), 0.01);
        assertEquals(438.91, HyperdriveUtil.convertDistance(4.8, Units.LENGTH.YARDS, Units.LENGTH.CENTIMETERS), 0.01);
        assertEquals(0.556, HyperdriveUtil.convertDistance(50.8, Units.LENGTH.CENTIMETERS, Units.LENGTH.YARDS), 0.01);
        assertEquals(0.81, HyperdriveUtil.convertDistance(81, Units.LENGTH.CENTIMETERS, Units.LENGTH.METERS), 0.01);
        assertEquals(725.0, HyperdriveUtil.convertDistance(7.25, Units.LENGTH.METERS, Units.LENGTH.CENTIMETERS), 0.01);
        assertEquals(2.388, HyperdriveUtil.convertDistance(94, Units.LENGTH.INCHES, Units.LENGTH.METERS), 0.01);
        assertEquals(2499.36, HyperdriveUtil.convertDistance(82, Units.LENGTH.FEET, Units.LENGTH.CENTIMETERS), 0.01);
        assertEquals(7.655, HyperdriveUtil.convertDistance(7, Units.LENGTH.METERS, Units.LENGTH.YARDS), 0.01);
        assertEquals(1.5, HyperdriveUtil.convertDistance(54, Units.LENGTH.INCHES, Units.LENGTH.YARDS), 0.01);

        assertEquals(0, HyperdriveUtil.convertDistance(0, Units.LENGTH.FEET, Units.LENGTH.FEET), 0.01);
        assertEquals(0, HyperdriveUtil.convertDistance(0, Units.LENGTH.FEET, Units.LENGTH.CENTIMETERS), 0.01);
        assertEquals(0, HyperdriveUtil.convertDistance(0, Units.LENGTH.FEET, Units.LENGTH.INCHES), 0.01);
        assertEquals(0, HyperdriveUtil.convertDistance(0, Units.LENGTH.FEET, Units.LENGTH.METERS), 0.01);
        assertEquals(0, HyperdriveUtil.convertDistance(0, Units.LENGTH.FEET, Units.LENGTH.YARDS), 0.01);
    }

    @Test
    public void testTimeUnits() {
        //time units(dsec, sec, min)
        assertEquals(10.0, HyperdriveUtil.convertTime(1, Units.TIME.SECONDS, Units.TIME.DECASECONDS), 0.01);
        assertEquals(6.6, HyperdriveUtil.convertTime(66, Units.TIME.DECASECONDS, Units.TIME.SECONDS), 0.01);
        assertEquals(0.5, HyperdriveUtil.convertTime(30, Units.TIME.SECONDS, Units.TIME.MINUTES), 0.01);
        assertEquals(45.0, HyperdriveUtil.convertTime(0.75, Units.TIME.MINUTES, Units.TIME.SECONDS), 0.01);
        assertEquals(1200.0, HyperdriveUtil.convertTime(2, Units.TIME.MINUTES, Units.TIME.DECASECONDS), 0.01);
        assertEquals(0.1, HyperdriveUtil.convertTime(60, Units.TIME.DECASECONDS, Units.TIME.MINUTES), 0.01);

        assertEquals(0, HyperdriveUtil.convertTime(0, Units.TIME.DECASECONDS, Units.TIME.DECASECONDS), 0.1);
        assertEquals(0, HyperdriveUtil.convertTime(0, Units.TIME.DECASECONDS, Units.TIME.SECONDS), 0.1);
        assertEquals(0, HyperdriveUtil.convertTime(0, Units.TIME.DECASECONDS, Units.TIME.MINUTES), 0.1);
    }

    @Test
    public void testForceUnits() {
        //force units (pounds, newtons, kilogram-force)
        assertEquals(22.241, HyperdriveUtil.convertForce(5, Units.FORCE.POUND, Units.FORCE.NEWTON), 0.01);
        assertEquals(1.416, HyperdriveUtil.convertForce(6.3, Units.FORCE.NEWTON, Units.FORCE.POUND), 0.01);
        assertEquals(0.306, HyperdriveUtil.convertForce(3, Units.FORCE.NEWTON, Units.FORCE.KILOGRAM_FORCE), 0.01);
        assertEquals(84.337, HyperdriveUtil.convertForce(8.6, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.NEWTON), 0.01);
        assertEquals(3.447, HyperdriveUtil.convertForce(7.6, Units.FORCE.POUND, Units.FORCE.KILOGRAM_FORCE), 0.01);
        assertEquals(5.622, HyperdriveUtil.convertForce(2.55, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.POUND), 0.01);

        assertEquals(0, HyperdriveUtil.convertForce(0, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.KILOGRAM_FORCE), 0.01);
        assertEquals(0, HyperdriveUtil.convertForce(0, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.NEWTON), 0.01);
        assertEquals(0, HyperdriveUtil.convertForce(0, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.POUND), 0.01);
    }

    @Test
    public void testMassAndWeight() {
        //converting from kg to N or LBF and whatever
        assertEquals(0, HyperdriveUtil.massKGFromWeight(0, Units.FORCE.KILOGRAM_FORCE), 0.01);
        assertEquals(0, HyperdriveUtil.massKGFromWeight(0, Units.FORCE.NEWTON), 0.01);
        assertEquals(0, HyperdriveUtil.massKGFromWeight(0, Units.FORCE.POUND), 0.01);

        assertEquals(0, HyperdriveUtil.weightFromMassKG(0, Units.FORCE.KILOGRAM_FORCE), 0.01);
        assertEquals(0, HyperdriveUtil.weightFromMassKG(0, Units.FORCE.NEWTON), 0.01);
        assertEquals(0, HyperdriveUtil.weightFromMassKG(0, Units.FORCE.POUND), 0.01);

        //convert 5 kilograms to weights and back
        assertEquals(49.03, HyperdriveUtil.weightFromMassKG(5, Units.FORCE.NEWTON), 0.01);
        assertEquals(11.0156, HyperdriveUtil.weightFromMassKG(5, Units.FORCE.POUND), 0.01);
        assertEquals(5, HyperdriveUtil.weightFromMassKG(5, Units.FORCE.KILOGRAM_FORCE), 0.01);

        assertEquals(5, HyperdriveUtil.massKGFromWeight(49.03, Units.FORCE.NEWTON), 0.01);
        assertEquals(5, HyperdriveUtil.massKGFromWeight(11.0156, Units.FORCE.POUND), 0.01);
        assertEquals(5, HyperdriveUtil.massKGFromWeight(5, Units.FORCE.KILOGRAM_FORCE), 0.01);
    }

    @Test
    public void testAngleBetweenHeadings() {
        //some zero tests fo today
        assertEquals(0, HyperdriveUtil.getAngleBetweenHeadings(0, 0), 0.01);
        assertEquals(0, HyperdriveUtil.getAngleBetweenHeadings(0, 360), 0.01);
        assertEquals(0, HyperdriveUtil.getAngleBetweenHeadings(360, 0), 0.01);
        assertEquals(0, HyperdriveUtil.getAngleBetweenHeadings(-720, 360), 0.01);

        //simple tests
        assertEquals(90, HyperdriveUtil.getAngleBetweenHeadings(0, 90), 0.01);
        assertEquals(60, HyperdriveUtil.getAngleBetweenHeadings(20, 80), 0.01);
        assertEquals(-90, HyperdriveUtil.getAngleBetweenHeadings(100, 10), 0.01);
        assertEquals(5, HyperdriveUtil.getAngleBetweenHeadings(265, 270), 0.01);
        assertEquals(8, HyperdriveUtil.getAngleBetweenHeadings(352, 360), 0.01);

        //complex tests
        assertEquals(-2, HyperdriveUtil.getAngleBetweenHeadings(0, 358), 0.01);
        assertEquals(-40, HyperdriveUtil.getAngleBetweenHeadings(20, 340), 0.01);
        assertEquals(-40, HyperdriveUtil.getAngleBetweenHeadings(20, -20), 0.01);
        assertEquals(-1, HyperdriveUtil.getAngleBetweenHeadings(1, 720), 0.01);
        assertEquals(1, HyperdriveUtil.getAngleBetweenHeadings(720, 1), 0.01);
    }

    @Test
    public void testValueImport() {
        double[] results1 = {
            2.1953320e+00,   
            1.0394349e+01,   
            6.7566305e+00,   
            2.0801780e+01,   
            1.1216760e+01,   
            2.3676837e+01,   
            4.7280227e+00,   
            6.8588758e+00
        };

        double[] results2 = {
            9.1247619e+00,   
            1.3344488e+01,   
            1.0447009e+01,   
            1.9751340e+00,   
            6.2378200e+00,   
            3.2062923e+00,   
            4.7816025e+00,  
            6.2387657e+00,
        };

        double[] results5 = {
            2.438796980967847,
            2.367631324097598,
            2.294259239780545,
            2.224953092550737,
            2.1331118916098375
        };

        assertArrayEquals(results1, HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport1.txt"), 0.01);
        assertArrayEquals(results2, HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport2.txt"), 0.01);
        assertArrayEquals(new double[0], HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport3.txt"), 0.01);
        assertArrayEquals(new double[0], HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport4.txt"), 0.01); //this one doesn't exist on purpose
        assertArrayEquals(results5, HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport5.txt"), 0.01);
    }

    @Test
    public void testGetDeviance() {
        //test 1 (points are right on top of each other, same heading, 0 deviance)
        Point2D
            current = new Point2D(6, 6, 8),
            target = new Point2D(6, 6, 8);

        assertEquals(0, HyperdriveUtil.getDeviance(current, target), 0.01);

        //test 2 (current point directly in front of target, same heading, 0 deviance)
        current = new Point2D(1, 2, 0);
        target  = new Point2D(3, 2, 0);
        assertEquals(0, HyperdriveUtil.getDeviance(current, target), 0.01);

        current = new Point2D(0, 0, 45);
        target = new Point2D(1, 1, 45);
        assertEquals(0, HyperdriveUtil.getDeviance(current, target), 0.01);

        //test 3 (current point directly beside target, same(ish) heading, easily calculated deviance);
        current = new Point2D(0, 0, 0);
        target = new Point2D(0, 1, 0);
        assertEquals(1, HyperdriveUtil.getDeviance(current, target), 0.01);

        current = new Point2D(5, 6, 180);
        target = new Point2D(5, 0, 180);
        assertEquals(6, HyperdriveUtil.getDeviance(current, target), 0.01);

        current = new Point2D(1, 1, 45); //45 degree offset should not matter because the robot is still on the perpendicular deviance line
        target = new Point2D(3, 1, 90);
        assertEquals(2, HyperdriveUtil.getDeviance(current, target), 0.01);

        //test 3 (more difficult cases)
        current = new Point2D(2, 5, 25);
        target = new Point2D(-1, -1, 0);
        assertEquals(4.601, HyperdriveUtil.getDeviance(current, target), 0.01);

        current = new Point2D(-8, 1, 180);
        target = new Point2D(-5, 6, 140);
        assertEquals(6.527, HyperdriveUtil.getDeviance(current, target), 0.01);

        current = new Point2D(9, 3, -69);
        target = new Point2D(5, 13, 37);
        assertEquals(0.547, HyperdriveUtil.getDeviance(current, target), 0.01);

        current = new Point2D(4, 4, 84);
        target = new Point2D(23, 18, 137);
        assertEquals(28.967, HyperdriveUtil.getDeviance(current, target), 0.01);
    }
}
