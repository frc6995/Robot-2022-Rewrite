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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Trajectories;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.subsystems.climb.SuperClimberS;
import frc.robot.util.NomadMathUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  @Log
  TurretS turretS;
  SuperClimberS superClimberS;
  @Log
  DrivebaseS drivebaseS;
  @Log
  ShooterS shooterS;
  @Log
  MidtakeS midtakeS;
  IntakeS intakeS;
  Limelight limelight;

  // The simulated field
  @Log
  Field2d field = new Field2d();

  Trigger shootButton = new Trigger(()->operatorController.getRightTriggerAxis() > 0.5);
  Trigger spinUpButton = new Trigger(()->operatorController.getLeftTriggerAxis() > 0.5);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    createSubsystems();
    createCommands();
    createTriggers();
    configureButtonBindings();
    // Configure the button bindings

  }

  private void createSubsystems() {
    turretS = new TurretS();
    superClimberS = new SuperClimberS();
    drivebaseS = new DrivebaseS();
    shooterS = new ShooterS();
    midtakeS = new MidtakeS();
    intakeS = new IntakeS();
    limelight = new Limelight();
  }

  private void createCommands() {
    turretS.setDefaultCommand(turretS.manualC(driverController::getRightX));
    drivebaseS.setDefaultCommand(
        drivebaseS.createCurvatureDriveC(
            () -> driverController.getRightTriggerAxis()-driverController.getLeftTriggerAxis(),
            driverController::getLeftX));
    midtakeS.setDefaultCommand(midtakeS.idleC());


  }

  private void createTriggers() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    driverController.a()
    .toggleWhenActive(
      new ParallelCommandGroup(
        intakeS.deployAndSpinC(),
        midtakeS.intakeC()
      )
    );

    spinUpButton.whileActiveContinuous(shooterS.spinVelocityC(()->3000, ()->3000));
    shootButton.whileActiveContinuous(midtakeS.shootC());

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
     * Set the robot position and the turret (as a rotated copy of the robot
     * position)
     */
    field.setRobotPose(drivebaseS.getRobotPose());
    field.getObject("turret").setPose(
        drivebaseS.getRobotPose()
            .transformBy(
                new Transform2d(new Translation2d(),
                    turretS.getRobotToTurretRotation())));

  }
}
