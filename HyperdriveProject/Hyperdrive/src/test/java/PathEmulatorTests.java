import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** {@link PathEmulator} tests */
public class PathEmulatorTests {

    /**
     * Test of the rough velocity mapping, the first part of the velocity mapping algorithm
     */
    @Test
    public void testRoughVelocityMapping() {
        //first path (short, going forward)
        Path path1 = new Path("src/test/java/files/velocitymap/path1.txt");
        double[] expectedResult1 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/roughresult1.txt");
        double[] actualResult1 = HyperdriveTestHelper.calculateRoughVelocityMapForPath(path1);

        assertArrayEquals(expectedResult1, actualResult1, 0.35);

        //second path (long, going forward)
        Path path2 = new Path("src/test/java/files/velocitymap/path2.txt");
        double[] expectedResult2 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/roughresult2.txt");
        double[] actualResult2 = HyperdriveTestHelper.calculateRoughVelocityMapForPath(path2);

        assertArrayEquals(expectedResult2, actualResult2, 0.35);

        //third path (short, going backwards)
        Path path3 = new Path("src/test/java/files/velocitymap/path3.txt");
        double[] expectedResult3 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/roughresult3.txt");
        double[] actualResult3 = HyperdriveTestHelper.calculateRoughVelocityMapForPath(path3);

        assertArrayEquals(expectedResult3, actualResult3, 0.35);

        //forth path (long, going backwards)
        Path path4 = new Path("src/test/java/files/velocitymap/path4.txt");
        double[] expectedResult4 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/roughresult4.txt");
        double[] actualResult4 = HyperdriveTestHelper.calculateRoughVelocityMapForPath(path4);

        assertArrayEquals(expectedResult4, actualResult4, 0.35);

        //fifth path (short, going both forwards and backwards)
        Path path5 = new Path("src/test/java/files/velocitymap/path5.txt");
        double[] expectedResult5 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/roughresult5.txt");
        double[] actualResult5 = HyperdriveTestHelper.calculateRoughVelocityMapForPath(path5);

        assertArrayEquals(expectedResult5, actualResult5, 0.35);

        //sixth path (long, going both forwards and backwards)
        Path path6 = new Path("src/test/java/files/velocitymap/path6.txt");
        double[] expectedResult6 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/roughresult6.txt");
        double[] actualResult6 = HyperdriveTestHelper.calculateRoughVelocityMapForPath(path6);

        assertArrayEquals(expectedResult6, actualResult6, 0.35);
    }

    /**
     * This Test compares values generated by the velocity-smoothing algorithm against values generated by the same 
     * algorithm programmed in MATLab. If the algorihtm should ever change, this test will make sure that the values 
     * produced are still valid.
     */
    @Test
    public void testVelocitySmoothing() {
        //first path (short, going forward)
        Path path1 = new Path("src/test/java/files/velocitymap/path1.txt");
        double[] expectedResult1 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/smoothedresult1.txt");
        double[] actualResult1 = HyperdriveTestHelper.calculateVelocityMapForPath(path1);

        assertArrayEquals(expectedResult1, actualResult1, 0.35);

        //second path (long, going forward)
        Path path2 = new Path("src/test/java/files/velocitymap/path2.txt");
        double[] expectedResult2 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/smoothedresult2.txt");
        double[] actualResult2 = HyperdriveTestHelper.calculateVelocityMapForPath(path2);

        assertArrayEquals(expectedResult2, actualResult2, 0.35);

        //third path (short, going backwards)
        Path path3 = new Path("src/test/java/files/velocitymap/path3.txt");
        double[] expectedResult3 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/smoothedresult3.txt");
        double[] actualResult3 = HyperdriveTestHelper.calculateVelocityMapForPath(path3);

        assertArrayEquals(expectedResult3, actualResult3, 0.35);

        //forth path (long, going backwards)
        Path path4 = new Path("src/test/java/files/velocitymap/path4.txt");
        double[] expectedResult4 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/smoothedresult4.txt");
        double[] actualResult4 = HyperdriveTestHelper.calculateVelocityMapForPath(path4);

        assertArrayEquals(expectedResult4, actualResult4, 0.35);

        //fifth path (short, going both forwards and backwards)
        Path path5 = new Path("src/test/java/files/velocitymap/path5.txt");
        double[] expectedResult5 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/smoothedresult5.txt");
        double[] actualResult5 = HyperdriveTestHelper.calculateVelocityMapForPath(path5);

        assertArrayEquals(expectedResult5, actualResult5, 0.35);

        //sixth path (long, going both forwards and backwards)
        Path path6 = new Path("src/test/java/files/velocitymap/path6.txt");
        double[] expectedResult6 = HyperdriveUtil.loadValuesFromFile("src/test/java/files/velocitymap/smoothedresult6.txt");
        double[] actualResult6 = HyperdriveTestHelper.calculateVelocityMapForPath(path6);

        assertArrayEquals(expectedResult6, actualResult6, 0.35);
    }
}
