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
import frc.robot.util.hyperdrive.util.HyperdriveUtil;
import frc.robot.util.hyperdrive.util.Path;
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

  /**
   * Tests some features.
   * @return True if the tests pass, false otherwise.
   */
  public boolean test() {
    return testUnitConversions();
  }

  /**
   * Tests all unit conversions using HyperdriveUtil. Returns the result of the test.
   * @return True if the tests passed, false otherwise.
   */
  private boolean testUnitConversions() {
    boolean allPassed = true;
    allPassed = allPassed && testUnits();
    
    return allPassed;
  }

  private boolean testUnits() {
    //distance units (in, ft, yd, cm, m)
    boolean allPassed = true;

    allPassed = allPassed && HyperdriveUtil.assertEquals("20 in = 1.1666 ft", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(20, Units.LENGTH.INCHES, Units.LENGTH.FEET), 3), 1.667); //20 inches = 1.666667 ft
    allPassed = allPassed && HyperdriveUtil.assertEquals("7.2 ft = 86.4 in", HyperdriveUtil.convertDistance(7.2, Units.LENGTH.FEET, Units.LENGTH.INCHES), 86.4);
    allPassed = allPassed && HyperdriveUtil.assertEquals("6.4 ft = 2.133 yd", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(6.4, Units.LENGTH.FEET, Units.LENGTH.YARDS), 3), 2.133);
    allPassed = allPassed && HyperdriveUtil.assertEquals("0.35 yd = 1.05 ft", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(0.35, Units.LENGTH.YARDS, Units.LENGTH.FEET), 3), 1.05);
    allPassed = allPassed && HyperdriveUtil.assertEquals("4.8 yd = 438.912 cm", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(4.8, Units.LENGTH.YARDS, Units.LENGTH.CENTIMETERS), 2), 438.91);
    allPassed = allPassed && HyperdriveUtil.assertEquals("50.8 cm = 0.555 yd", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(50.8, Units.LENGTH.CENTIMETERS, Units.LENGTH.YARDS), 3), 0.556);
    allPassed = allPassed && HyperdriveUtil.assertEquals("81 cm = 0.81 m", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(81, Units.LENGTH.CENTIMETERS, Units.LENGTH.METERS), 2), 0.81);
    allPassed = allPassed && HyperdriveUtil.assertEquals("7.25 m = 725.0 cm", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(7.25, Units.LENGTH.METERS, Units.LENGTH.CENTIMETERS), 1), 725.0);
    allPassed = allPassed && HyperdriveUtil.assertEquals("94 in = 2.3876 m", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(94, Units.LENGTH.INCHES, Units.LENGTH.METERS), 3), 2.388);
    allPassed = allPassed && HyperdriveUtil.assertEquals("82 ft = 2499.36 cm", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(82, Units.LENGTH.FEET, Units.LENGTH.CENTIMETERS), 2), 2499.36);
    allPassed = allPassed && HyperdriveUtil.assertEquals("7 m = 7.655 yd", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(7, Units.LENGTH.METERS, Units.LENGTH.YARDS), 3), 7.655);
    allPassed = allPassed && HyperdriveUtil.assertEquals("54 in = 1.5 yd", HyperdriveUtil.roundTo(HyperdriveUtil.convertDistance(54, Units.LENGTH.INCHES, Units.LENGTH.YARDS), 1), 1.5);

    //time units(dsec, sec, min)
    allPassed = allPassed && HyperdriveUtil.assertEquals("1 sec = 10 dsec", HyperdriveUtil.convertTime(1, Units.TIME.SECONDS, Units.TIME.DECASECONDS), 10.0);
    allPassed = allPassed && HyperdriveUtil.assertEquals("66 dsec = 6.6 sec", HyperdriveUtil.convertTime(66, Units.TIME.DECASECONDS, Units.TIME.SECONDS), 6.6);
    allPassed = allPassed && HyperdriveUtil.assertEquals("30 sec = 0.5 min", HyperdriveUtil.roundTo(HyperdriveUtil.convertTime(30, Units.TIME.SECONDS, Units.TIME.MINUTES), 1), 0.5);
    allPassed = allPassed && HyperdriveUtil.assertEquals("0.75 min = 45 sec", HyperdriveUtil.roundTo(HyperdriveUtil.convertTime(0.75, Units.TIME.MINUTES, Units.TIME.SECONDS), 3), 45.0);
    allPassed = allPassed && HyperdriveUtil.assertEquals("2 min = 1200 dsec", HyperdriveUtil.roundTo(HyperdriveUtil.convertTime(2, Units.TIME.MINUTES, Units.TIME.DECASECONDS), 3), 1200.0);
    allPassed = allPassed && HyperdriveUtil.assertEquals("60 dsec = 0.1 min", HyperdriveUtil.roundTo(HyperdriveUtil.convertTime(60, Units.TIME.DECASECONDS, Units.TIME.MINUTES), 3), 0.1);

    //force units (pounds, newtons, kilogram-force)
    allPassed = allPassed && HyperdriveUtil.assertEquals("5 lb = 22.241 N", HyperdriveUtil.roundTo(HyperdriveUtil.convertForce(5, Units.FORCE.POUND, Units.FORCE.NEWTON), 3), 22.241);
    allPassed = allPassed && HyperdriveUtil.assertEquals("6.3 N = 1.416 N", HyperdriveUtil.roundTo(HyperdriveUtil.convertForce(6.3, Units.FORCE.NEWTON, Units.FORCE.POUND), 3), 1.416);
    allPassed = allPassed && HyperdriveUtil.assertEquals("3 N = 0.306 kg", HyperdriveUtil.roundTo(HyperdriveUtil.convertForce(3, Units.FORCE.NEWTON, Units.FORCE.KILOGRAM_FORCE), 3), 0.306);
    allPassed = allPassed && HyperdriveUtil.assertEquals("8.6 kg = 84.337 N", HyperdriveUtil.roundTo(HyperdriveUtil.convertForce(8.6, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.NEWTON), 3), 84.337);
    allPassed = allPassed && HyperdriveUtil.assertEquals("7.6 lb = 3.447 kg", HyperdriveUtil.roundTo(HyperdriveUtil.convertForce(7.6, Units.FORCE.POUND, Units.FORCE.KILOGRAM_FORCE), 3), 3.447);
    allPassed = allPassed && HyperdriveUtil.assertEquals("2.55 kg = 5.622 lb", HyperdriveUtil.roundTo(HyperdriveUtil.convertForce(2.55, Units.FORCE.KILOGRAM_FORCE, Units.FORCE.POUND), 3), 5.622);

    return allPassed;
  }
}
