// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.commands.CyborgCommandRecordPath;
import frc.robot.commands.CyborgCommandTestVelocity;
import frc.robot.subsystems.SubsystemDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final SubsystemDrive SUB_DRIVE = new SubsystemDrive();

  // Controllers
  private static final Joystick DRIVER = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Zeros the drivetrain.
   */
  private void zeroDrivetrain() {
    SUB_DRIVE.zeroDrivetrain();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    SUB_DRIVE.setDefaultCommand(new RunCommand( () -> { SUB_DRIVE.driveManually(DRIVER); }, SUB_DRIVE));

    InstantCommand zeroDrivetrain = new InstantCommand( () -> { zeroDrivetrain(); } );
    SmartDashboard.putData(zeroDrivetrain);    

    SmartDashboard.putData(new CyborgCommandTestVelocity(SUB_DRIVE));
    SmartDashboard.putData(new CyborgCommandRecordPath(SUB_DRIVE));
    SmartDashboard.putData(new CyborgCommandEmulatePath(SUB_DRIVE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}


