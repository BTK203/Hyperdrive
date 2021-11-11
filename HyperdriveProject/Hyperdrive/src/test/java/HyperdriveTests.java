// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Units;

/** 
 * Test class for some foundational Hyperdrive structures.
 */
public class HyperdriveTests {
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

        assertArrayEquals(results1, HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport1.txt"), 0.01);
        assertArrayEquals(results2, HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport2.txt"), 0.01);
        assertArrayEquals(new double[0], HyperdriveUtil.loadValuesFromFile("src/test/java/files/valueImport/valueimport3.txt"), 0.01);
        assertArrayEquals(new double[0], HyperdriveUtil.loadValuesFromFile("src/text/java/files/valurImport/valueimport4.txt"), 0.01); //this one doesn't exist on purpose
    }

    /**
     * This Test compares values generated by the velocity-smoothing algorithm against values generated by the same 
     * algorithm programmed in MATLab. If the algorihtm should ever change, this test will make sure that the values 
     * produced are still valid.
     */
    @Test
    public void testVelocityMapping() {
        //first path (short, going forward)
        Path path1 = new Path("src/test/java/files/velocitymap/path1.txt");
        double[] expectedResult1 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/matresult1.txt");
        double[] actualResult1 = HyperdriveTestHelper.calculateVelocityMapForPath(path1);

        assertArrayEquals(expectedResult1, actualResult1, 0.35);

        //second path (long, going forward)
        Path path2 = new Path("src/test/java/files/velocitymap/path2.txt");
        double[] expectedResult2 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/matresult2.txt");
        double[] actualResult2 = HyperdriveTestHelper.calculateVelocityMapForPath(path2);

        assertArrayEquals(expectedResult2, actualResult2, 0.35);

        //third path (short, going backwards)
        Path path3 = new Path("src/test/java/files/velocitymap/path3.txt");
        double[] expectedResult3 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/matresult3.txt");
        double[] actualResult3 = HyperdriveTestHelper.calculateVelocityMapForPath(path3);

        assertArrayEquals(expectedResult3, actualResult3, 0.35);

        //forth path (long, going backwards)
        Path path4 = new Path("src/test/java/files/velocitymap/path4.txt");
        double[] expectedResult4 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/matresult4.txt");
        double[] actualResult4 = HyperdriveTestHelper.calculateVelocityMapForPath(path4);

        assertArrayEquals(expectedResult4, actualResult4, 0.35);

        //fifth path (short, going both forwards and backwards)
        Path path5 = new Path("src/test/java/files/velocitymap/path5.txt");
        double[] expectedResult5 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/matresult5.txt");
        double[] actualResult5 = HyperdriveTestHelper.calculateVelocityMapForPath(path5);

        assertArrayEquals(expectedResult5, actualResult5, 0.35);

        //sixth path (long, going both forwards and backwards)
        Path path6 = new Path("src/test/java/files/velocitymap/path6.txt");
        double[] expectedResult6 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/matresult6.txt");
        double[] actualResult6 = HyperdriveTestHelper.calculateVelocityMapForPath(path6);

        assertArrayEquals(expectedResult6, actualResult6, 0.35);
    }

    /**
     * TODO: Add tests for TankTrajectory
     */
}
