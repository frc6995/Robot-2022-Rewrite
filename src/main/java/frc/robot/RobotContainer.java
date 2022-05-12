// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretS;
import io.github.oblarg.oblog.Loggable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
  CommandXboxController driverController = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  TurretS turretS;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    createSubsystems();
    createCommands();
    createTriggers();
    configureButtonBindings();
    // Configure the button bindings
    
  }

  private void createSubsystems() {
    turretS = new TurretS();
  }

  private void createCommands() {
    turretS.setDefaultCommand(turretS.createManualC(driverController::getLeftX));
  }

  private void createTriggers() {
  }







  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.a().whenActive(turretS.createFollowC(()->Constants.SOFT_LIMIT_REVERSE_RADIAN));
    driverController.b().whenActive(turretS.createMinimizeErrorC(()->{return turretS.getError(new Rotation2d(Math.PI));}));
    driverController.x().whenActive(turretS.createFollowC(()->Constants.SOFT_LIMIT_FORWARD_RADIAN));
    
    driverController.y().whenActive(turretS.getDefaultCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
