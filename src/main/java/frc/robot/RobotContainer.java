// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.Trajectories;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.NomadMathUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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
  DrivebaseS drivebaseS;

  // The simulated field
  @Log
  Field2d field = new Field2d();
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
    drivebaseS = new DrivebaseS();
  }

  private void createCommands() {
    turretS.setDefaultCommand(turretS.manualC(driverController::getRightX));
    drivebaseS.setDefaultCommand(
      drivebaseS.createCurvatureDriveC(
        ()-> -driverController.getLeftY(),
        driverController::getLeftX
      )
    );
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
    driverController.a().whenActive(turretS.turnAngleC(()->{
      return NomadMathUtil.getDirection(new Transform2d(drivebaseS.getRobotPose(), Trajectories.HUB_CENTER_POSE)).getRadians();
    }));
    driverController.b().whenActive(turretS.zeroErrorC(()->{return turretS.getError(new Rotation2d(Math.PI));}));
    driverController.x().whenActive(turretS.turnAngleC(()->3*Math.PI/2));
    
    driverController.y().whenActive(turretS.turnAngleC(()->{return Units.degreesToRadians(180 + (driverController.getLeftX() * 110));}));
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

  public void periodic() {
    /**
     * Set the robot position and the turret (as a rotated copy of the robot position)
     */
    field.setRobotPose(drivebaseS.getRobotPose());
    field.getObject("turret").setPose(
      drivebaseS.getRobotPose()
      .transformBy(
        new Transform2d(new Translation2d(),
          turretS.getRobotToTurretRotation()
        )
      )
    );

  }
}
