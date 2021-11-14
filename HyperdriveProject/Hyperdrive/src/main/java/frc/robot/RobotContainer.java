// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.commands.CyborgCommandRecordPath;
import frc.robot.commands.InstantCommandCreateRandomPath;
import frc.robot.commands.InstantCommandSetRobotPosition;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.util.hyperdrive.emulation.PreferenceEmulationParams;
import frc.robot.util.hyperdrive.util.Units;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SubsystemDrive drivetrain = new SubsystemDrive();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //default driving command
    drivetrain.setDefaultCommand(new RunCommand( () -> { drivetrain.driveWithKeyboardInput(); }, drivetrain));

    //dashboard commands
    InstantCommand setRobotPositionToPathStart = new InstantCommand( () -> { drivetrain.setPoseToRecordedPathStart(); } );
    SmartDashboard.putData("Set Robot To Start", setRobotPositionToPathStart);
    SmartDashboard.putData("Emulate", new CyborgCommandEmulatePath(drivetrain, drivetrain.getHyperdrive()));
    SmartDashboard.putData("Emulate with Test Params", new CyborgCommandEmulatePath(drivetrain, drivetrain.getHyperdrive(), Constants.TEST_PARAMS));
    SmartDashboard.putData("Emulate Benchmark", new CyborgCommandEmulatePath(drivetrain, drivetrain.getHyperdrive(), new PreferenceEmulationParams(Units.LENGTH.METERS), "benchmark.txt"));
    SmartDashboard.putData("Record", new CyborgCommandRecordPath(drivetrain.getHyperdrive()));
    SmartDashboard.putData("Create Random", new InstantCommandCreateRandomPath("src/main/java/frc/robot/random.txt", drivetrain.getHyperdrive()));
    SmartDashboard.putData("Zero Coords", new InstantCommand(() -> drivetrain.zeroPositionAndHeading()));
    SmartDashboard.putData("Set Pos", new InstantCommandSetRobotPosition(drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
