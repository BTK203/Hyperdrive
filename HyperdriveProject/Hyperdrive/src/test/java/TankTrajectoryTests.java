import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.Constants;
import frc.robot.util.hyperdrive.emulation.IController;
import frc.robot.util.hyperdrive.emulation.TankController;
import frc.robot.util.hyperdrive.emulation.TankTrajectory;
import frc.robot.util.hyperdrive.emulation.Trajectory;
import frc.robot.util.hyperdrive.util.Units;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Test class for the {@link TankTrajectory} class. */
public class TankTrajectoryTests {
    public double TEST_MOTOR_UNIT_SCALAR = 0.5;

    /**
     * Tests converting between actual units and motor units
     */
    @Test
    public void testMotorScalarConversion() {
        final double regularVelocity = 1;
        IController controller = new TankController(Constants.TEST_PARAMS);
        Trajectory base = new Trajectory(regularVelocity, 1, 0, Constants.TEST_PARAMS, controller, TEST_MOTOR_UNIT_SCALAR);
        TankTrajectory tt = new TankTrajectory(base, Constants.WHEEL_BASE_WIDTH);

        final double expected = regularVelocity * TEST_MOTOR_UNIT_SCALAR;
        assertEquals(expected, tt.getLeftVelocity(), 0.1);
        assertEquals(expected, tt.getRightVelocity(), 0.1);
    }

    /**
     * Tests time conversion in tanktrajectories.
     * NOTE: This test assumes that the motor scalar conversion test has passed.
     */
    @Test 
    public void testConvertTime() {
        IController controller = new TankController(Constants.TEST_PARAMS);
        Trajectory base = new Trajectory(1, 1, (Math.PI / 2), Constants.TEST_PARAMS, controller, TEST_MOTOR_UNIT_SCALAR);

        //units per second to units per second
        TankTrajectory tt1 = new TankTrajectory(base, Constants.WHEEL_BASE_WIDTH);
        tt1.convertTime(Units.TIME.SECONDS); //should be exactly the same after this call
        
        final double 
            leftExpected = 0.60730091 * TEST_MOTOR_UNIT_SCALAR,
            rightExpected = 1.39169981 * TEST_MOTOR_UNIT_SCALAR;

        assertEquals(leftExpected, tt1.getLeftVelocity(), 0.1);
        assertEquals(rightExpected, tt1.getRightVelocity(), 0.1);

        //units per second to units per decasecond
        TankTrajectory tt2 = new TankTrajectory(base, Constants.WHEEL_BASE_WIDTH);
        tt2.convertTime(Units.TIME.DECASECONDS);

        final double
            leftExpected2 = 0.060730091 * TEST_MOTOR_UNIT_SCALAR,
            rightExpected2 = 0.1391699081 * TEST_MOTOR_UNIT_SCALAR;

        assertEquals(leftExpected2, tt2.getLeftVelocity(), 0.1);
        assertEquals(rightExpected2, tt2.getRightVelocity(), 0.1);

        //units per second to units per minute
        TankTrajectory tt3 = new TankTrajectory(base, Constants.WHEEL_BASE_WIDTH);
        tt3.convertTime(Units.TIME.MINUTES);

        final double 
            leftExpected3 = 36.4380546 * TEST_MOTOR_UNIT_SCALAR,
            rightExpected3 = 83.50194486 * TEST_MOTOR_UNIT_SCALAR;

        assertEquals(leftExpected3, tt3.getLeftVelocity(), 0.1);
        assertEquals(rightExpected3, tt3.getRightVelocity(), 0.1);
    }

    /**
     * Tests the output of the built-in {@link IController} in the tank trajectory.
     */
    @Test
    public void testControllerOutput() {
        IController controller = new TankController(Constants.TEST_PARAMS);
        Trajectory base = new Trajectory(1, 1, (Math.PI / 2), Constants.TEST_PARAMS, controller, TEST_MOTOR_UNIT_SCALAR);
        TankTrajectory tt = new TankTrajectory(base, Constants.WHEEL_BASE_WIDTH);

        //first pass
        final double
            leftCurrent = 0.25,
            rightCurrent = 0.6,
            leftExpected = 0.139485,
            rightExpected = 0.1510143;

        assertEquals(leftExpected, tt.getLeftPercentOutput(leftCurrent), 0.1);
        assertEquals(rightExpected, tt.getRightPercentOutput(rightCurrent), 0.1);

        //second pass (mainly for I and D)
        final double
            leftExpected2 = 0.1405586,
            rightExpected2 = 0.152941;

        assertEquals(leftExpected2, tt.getLeftPercentOutput(leftCurrent), 0.1);
        assertEquals(rightExpected2, tt.getRightPercentOutput(rightCurrent), 0.1);
    }
}
