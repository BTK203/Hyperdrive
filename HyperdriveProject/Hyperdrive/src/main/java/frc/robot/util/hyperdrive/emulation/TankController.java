// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.emulation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;

/** 
 * Controller for tank-style robot.
 */
public class TankController implements IController {
    public static final int
        LEFT_CONTROLLER = 0,
        RIGHT_CONTROLLER = 1;

    private PIDController[] controllers;
    private IEmulateParams parameters;

    public TankController(IEmulateParams parameters) {
        this.parameters = parameters;
        controllers = new PIDController[2]; // two PIDControllers; one for each side
        
        //grab kP, kI, and kD values from parameters
        double
            kP = parameters.getPIDFAConfig().getkP(),
            kI = parameters.getPIDFAConfig().getkI(),
            kD = parameters.getPIDFAConfig().getkD();

        controllers[0] = new PIDController(kP, kI, kD);
        controllers[1] = new PIDController(kP, kI, kD);
    }

    public PIDController getPIDController(int id) {
        if(id >= controllers.length) {
            DriverStation.reportError("PID Controller with id " + id + " does not exist! Returning \"safe\" PIDController.", true);
            return new PIDController(0, 0, 0);
        }

        return controllers[id];
    }

    public double calculateOutput(int controllerID, double setpoint, double current) {
        //if setpoint needs to be changed, change it
        if(controllers[controllerID].getSetpoint() != setpoint) {
            controllers[controllerID].setSetpoint(setpoint);
        }

        //calculate output and apply feed-forward
        double output = controllers[controllerID].calculate(current);
        output += parameters.getPIDFAConfig().getkF();

        //make sure output is within bounds
        if(output < parameters.getPIDFAConfig().getMinimumOutput()) {
            output = parameters.getPIDFAConfig().getMinimumOutput();
        } else if(output > parameters.getPIDFAConfig().getMaximumOutput()) {
            output = parameters.getPIDFAConfig().getMaximumOutput();
        }

        return output;
    }
}
