package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.auto.Trajectories;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.NomadMathUtil;


public class AutoCommandFactory {
    /**
     * Creates a command group to complete a two ball auto: spins up the shooter,
     * sets the front and
     * back shooter motors, homes the turret, drives, intakes one ball, retracts the
     * intake, and spins midtake to
     * shoot two balls.
     * 
     * @param targetFrontSpeed the target speed of the front shooter motor
     * @param targetBackSpeed  the target speed of the back shooter motor
     * @param targetAngle      the target angle of the turret
     * @param shooterS         the shooter subsystem
     * @param intakeS          the intake subsystem
     * @param midtakeS         the midtake subsystem
     * @param turretS          the turret subsystem
     * @param drivebaseS       the drivebase subsystem
     * @return the two ball auto command group
     */
    public static Command twoBallAutoC(
            ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS, LimelightS limelightS,
            DrivebaseS drivebaseS) {
        return
        new InstantCommand(
        () -> {
          drivebaseS.resetRobotPose(Trajectories.MID_BALL_START_POSE);
        })
        .andThen(
         new ParallelCommandGroup(
            shooterS.spinVelocityC(()->1750, ()->1750),
            new ParallelCommandGroup(
                drivebaseS.timedDriveC(0.4, 1.5),
                new ParallelCommandGroup(midtakeS.intakeC(), intakeS.deployAndSpinC())
                    .withTimeout(2.25)
            ).andThen(
                midtakeS.shootC(shooterS.atTargetTrigger).withTimeout(5)
            )
        )
        )
        .withName("Two Ball Auto");
    }


  public static Command fourBallAutoTrajectoryC(ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS,
      TurretS turretS, LimelightS limelightS, DrivebaseS drivebaseS) {
    return new InstantCommand(
        () -> {
          drivebaseS.resetRobotPose(Trajectories.MID_BALL_START_POSE);
        })
    .andThen(
        new ParallelCommandGroup(
            shooterS.spinVelocityC(() -> (1700), () -> (1700)),
            turretS.aimWithLimelight(limelightS::getFilteredXOffset, ()->0),
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        drivebaseS.ramseteC(Trajectories.MID_START_TO_MID_RING),
                        midtakeS.intakeC(),
                        intakeS.deployAndSpinC()
                ),
                midtakeS.shootC(shooterS.atTargetTrigger).withTimeout(3),
                        // End Two Ball Auto
                        // Turn a little bit
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        drivebaseS.ramseteC(Trajectories.MID_RING_TO_TERMINAL_PICKUP),
                        drivebaseS.timedDriveC(-0.2, 0.5),
                        new WaitCommand(0.25),
                        drivebaseS.ramseteC(Trajectories.TERMINAL_RECEIVE_TO_MID_RING)
                    ),
                    midtakeS.intakeC(),
                    intakeS.deployAndSpinC()
                ),
                midtakeS.shootC(shooterS.atTargetTrigger).withTimeout(3)
            )
        )
    );                // Intake Second Ball     
  }
}
