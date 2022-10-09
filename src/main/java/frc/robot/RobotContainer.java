// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoCommandFactory;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightS;
import frc.robot.subsystems.LightS.States;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.subsystems.climb.SuperClimberS;
import frc.robot.util.command.TriggerUtil;
import frc.robot.util.interpolation.ShooterInterpolatingTable;
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
  @Log
  DrivebaseS drivebaseS;
  @Log
  ShooterS shooterS;
  @Log
  MidtakeS midtakeS;
  IntakeS intakeS;
  SuperClimberS climberS;
  LimelightS limelightS;
  LightS lightS;

  // The simulated field
  @Log
  Field2d field = new Field2d();

  Trigger
  shootButton,
  spinUpButton,
  intakeToggleButton,
  climberTiltForwardButton,
  climberTiltBackButton,
  climberExtendFrontButton,
  climberExtendBackButton,
  climberRetractFrontButton,
  climberRetractBackButton,
  climberExtendBothButton,
  climberTransferButton,
  climberLockButton,
  climbIsLockedTrigger;

  DoubleSupplier
  driveForwardAxis,
  driveBackwardAxis,
  driveSteerAxis,
  turretManualAxis;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Startup order is important.
    // Subsystems first, because triggers and commands both depend on subsystems
    createSubsystems();
    createTriggers();
    createCommands();
    // button bindings last because those need both subsystems and triggers
    configureButtonBindings();
    // start the USB camera
    CameraServer.startAutomaticCapture();
    // Configure the button bindings

  }

  private void createSubsystems() {
    turretS = new TurretS();
    climberS = new SuperClimberS();
    drivebaseS = new DrivebaseS();
    shooterS = new ShooterS();
    midtakeS = new MidtakeS();
    intakeS = new IntakeS();
    limelightS = new LimelightS();
    lightS = LightS.getInstance();
  }

  private void createCommands() {
    turretS.setDefaultCommand(turretS.manualC(operatorController::getRightX));
    drivebaseS.setDefaultCommand(
        drivebaseS.createCurvatureDriveC(
            () -> driverController.getRightTriggerAxis()-driverController.getLeftTriggerAxis(),
            driverController::getLeftX));
    midtakeS.setDefaultCommand(midtakeS.idleC());


  }

  /**
   * This method exists to assign names to the controls independent from their bindings. This is easy for drivers to read.
   */
  private void createTriggers() {
    spinUpButton = TriggerUtil.asButton(operatorController::getLeftTriggerAxis, 0.5);
    shootButton = TriggerUtil.asButton(operatorController::getRightTriggerAxis, 0.5);

    intakeToggleButton = driverController.a();

    climberTiltForwardButton = driverController.pov.left();
    climberTiltBackButton = driverController.pov.right();

    climberExtendBackButton = operatorController.pov.up();
    climberRetractBackButton = operatorController.pov.down();

    climberExtendFrontButton = operatorController.y();
    climberRetractFrontButton = operatorController.a();

    climberExtendBothButton = operatorController.x();
    climberTransferButton = operatorController.b();

    climberLockButton = operatorController.start();

    driveForwardAxis = driverController::getRightTriggerAxis;
    driveBackwardAxis = driverController::getLeftTriggerAxis;
    driveSteerAxis = driverController::getLeftX;
    turretManualAxis = operatorController::getRightX;
    
    climbIsLockedTrigger = new Trigger(climberS::getIsLocked);
  }

  private void configureButtonBindings() {
    // INTAKE

    // We create a trigger that toggles true-false when Driver A is pressed
    intakeToggleButton
    // when that toggled trigger is true AND (shootButton is NOT true)
    // i.e., when intaking and not shooting
    .and(shootButton.negate())
    .whileActiveContinuous(
      // Requires: intakeS, midtakeS
      new ParallelCommandGroup(
        intakeS.deployAndSpinC(),
        midtakeS.intakeC(),
        lightS.requestStateC(()->States.Intaking)
      )
    );

    // SPIN UP AND AIM

    spinUpButton.whileActiveContinuous(
      // Requires: shooterS, turretS (limelightS is required separately)
      new ParallelCommandGroup(
        shooterS.spinDistanceC(limelightS::getFilteredDistance),
        limelightS.ledsOnC(), // will automatically turn off after command
        turretS.aimWithLimelight(
          ()->-limelightS.getFilteredXOffset(), 
          ()->MathUtil.applyDeadband(-operatorController.getRightX(), 0.05)
        ),
        lightS.requestStateC(
          ()-> {
            if(shooterS.isAtTarget()) {
              // if distance is in achievable range
              if (ShooterInterpolatingTable.getInRange(limelightS.getFilteredDistance())) {
                return States.ShooterAndDistanceReady;
              }
              else { return States.ShooterReady; }
            }
            else { return States.Shooting; }
          }
        )
      )
    );
    // Pulses the midtake while the shoot trigger is held down.
    // Requires: midtakeS
    shootButton.whileActiveContinuous(midtakeS.shootC(shooterS.atTargetTrigger));

    // Climber lock on turret, with accompanying lights.
    // We set this whileActiveOnce so that it can be interrupted (i.e. by spinup), but won't go back to the default command.
    climbIsLockedTrigger.whileActiveOnce(
      // Requires: turretS (not lightS)
      new ParallelCommandGroup(
        turretS.turnAngleC(()->Math.PI),
        lightS.requestStateC(()->States.Climbing)));
      
    createClimberCommandBindings();
  }

  public void createClimberCommandBindings() {
    
    // POV is the D-pad
    climberTiltForwardButton.whileActiveContinuous(climberS.forwardTiltC());
    climberTiltBackButton.whileActiveContinuous(climberS.backwardTiltC());

    climberExtendBackButton.whileActiveContinuous(climberS.extendBackC());
    climberRetractBackButton.whileActiveContinuous(climberS.retractBackC());

    climberExtendBothButton.whileActiveContinuous(climberS.extendBackC().alongWith(climberS.extendFrontC()));
    climberExtendFrontButton.whileActiveContinuous(climberS.extendFrontC());
    climberRetractFrontButton.whileActiveContinuous(climberS.retractFrontC());
    climberTransferButton.whileActiveContinuous(climberS.transferBackC());

    // toggle climber lock when pressing operator start
    climberLockButton.toggleWhenActive(new StartEndCommand(climberS::lock, climberS::unlock));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return AutoCommandFactory.twoBallAutoC(shooterS, intakeS, midtakeS, turretS, limelightS, drivebaseS);
  }

  public void periodic() {


    if (DriverStation.isDisabled()) {
      LightS.getInstance().requestState(States.Disabled);
    }

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
