// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;
import edu.wpi.first.wpilibj.controller.PIDController;

/** 
 * Interface for objects containing multiple PID controllers and other important runtime parameters.
 */
public interface IController {
    /**
     * Returns the PIDController with the given name. "Index" values will be provided through implementing
     * classes. For example, if using a {@link TankController}, possible values are 
     * LEFT_CONTROLLER and RIGHT_CONTROLLER.
     * @param id The ID of the controller to use. As mentioned earlier, values will be provided as static
     * fields by the implementing class.
     * @return The PIDController with the specified identity, or a "safe" PIDController (where all gains are 
     * set to 0) if there is no such controller.
     */
    public PIDController getPIDController(int id);

    /**
     * Uses the specified controller to calculate the necessary percent output to acheive a value.
     * @param controllerID The ID of the controller to use. For more information about IDs, see
     * {@link #getPIDController(int)}.
     * @param setpoint The desired value to acheive.
     * @param current The current value.
     * @return The output required to drive the "current" value to the "desired" value.
     */
    public double calculateOutput(int controllerID, double setpoint, double current);
}
