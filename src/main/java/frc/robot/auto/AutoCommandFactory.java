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
            new ParallelDeadlineGroup(
                // Limelight on
                // turret aim
                //shooter interpolate
               
                // run trajectory 
                new ParallelDeadlineGroup(
                    drivebaseS.timedDriveC(0.2, 1.25),
                    intakeS.deployC()
                ).andThen(new WaitCommand(0.5)).andThen(            
                    new ParallelCommandGroup(
                        midtakeS.shootC(shooterS.atTargetTrigger), 
                        intakeS.deployAndSpinC()
                    ).withTimeout(3.25)
                ),
                limelightS.ledsOnC(),
                turretS.aimWithLimelight(()->0 , ()->0),
                shooterS.spinDistanceC(()->3.7)
                        
                )
            )
        .withName("Two Ball Auto");
    }


  public static Command fourBallAutoTrajectoryC(ShooterS shooterS, IntakeS intakeS, 
  MidtakeS midtakeS, TurretS turretS, LimelightS limelightS, DrivebaseS drivebaseS) {
    return new InstantCommand(
        () -> {
          drivebaseS.resetRobotPose(Trajectories.FOUR_BALL_BACKUP_ONE.getInitialPose());
        })
    .andThen(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // First path while deploying (not spinning) intake
                new ParallelDeadlineGroup(
                    drivebaseS.ramseteC(Trajectories.FOUR_BALL_BACKUP_ONE),
                    intakeS.deployC()
                ).andThen(new WaitCommand(0.5)).andThen(
                    // Empty the midtake while bringing the third ball into the 
                    new ParallelCommandGroup(
                        midtakeS.shootC(shooterS.atTargetTrigger), 
                        intakeS.deployAndSpinC()
                    ).withTimeout(2.75)
                ),
                
                // Terminal retrieve and return while intaking
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        drivebaseS.ramseteC(Trajectories.FOUR_BALL_BACKUP_TWO),
                        drivebaseS.timedDriveC(-0.2, 0.5),
                        new WaitCommand(0.25),
                        drivebaseS.ramseteC(Trajectories.FOUR_BALL_RETURN)
                    ),
                    midtakeS.intakeC(),
                    intakeS.deployAndSpinC()
                ),
                midtakeS.shootC(shooterS.atTargetTrigger).withTimeout(2)
            ),

            shooterS.spinDistanceC(
                ()->NomadMathUtil.getDistance(
                    new Transform2d(Trajectories.FOUR_BALL_BACKUP_TWO.getInitialPose(), Trajectories.HUB_CENTER_POSE)
                )
            ),
            limelightS.ledsOnC(),
            turretS.aimWithLimelight(()->{
                var tail = Trajectories.FOUR_BALL_BACKUP_TWO.getInitialPose();
                var head = Trajectories.HUB_CENTER_POSE;
                var currentTurretFieldRotation = turretS.getRobotToTurretRotation().plus(Trajectories.FOUR_BALL_BACKUP_TWO.getInitialPose().getRotation());
                var targetTurretFieldRotation = 
                    new Rotation2d(head.getX()-tail.getX(), head.getY()-tail.getY()).plus(Rotation2d.fromDegrees(5)).unaryMinus();
                return targetTurretFieldRotation.plus(currentTurretFieldRotation).getRadians();
            }, ()->0)
        )
    );           
  }
}
