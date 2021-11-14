import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.Test;

import frc.robot.Constants;
import frc.robot.util.hyperdrive.util.TankGyro;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Test class for the {@link TankGyro} class.  */
public class TankGyroTests {
    @Test
    public void testGyro() {
        TankGyro gyro = new TankGyro(Constants.WHEEL_BASE_WIDTH, 1);
        gyro.update(0, 0);

        try {
            Thread.sleep(20);
        } catch(InterruptedException ex) {
            fail("Encountered an InterruptedException!");
        }

        gyro.update(0, (Math.PI * Constants.WHEEL_BASE_WIDTH));
        
        final double expected = 45; // drift compensation reduces actual value to compensate for acceleration
        double actual = gyro.getHeading();

        assertEquals(expected, actual, 0.01);
    }
}
