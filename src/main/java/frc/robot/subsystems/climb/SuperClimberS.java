// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsManager;
import frc.robot.subsystems.LightsManager.States;
import frc.robot.util.command.RunEndCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SuperClimberS implements Loggable {

  public final ThriftyClimberS thriftyClimberS = new ThriftyClimberS();
  public final LinearClimberS linearClimberS = new LinearClimberS();
  public final TiltClimberS tiltClimberS = new TiltClimberS();

  @Log
  private boolean climberLocked = true;
  
  /** Creates a new ClimberS. */
  public SuperClimberS() {
    // We make the default commands perpetual,
    // overriding isFinished to never end unless interrupted.
    // Default commands are not allowed to end naturally.
    // When movement commands for each subsystem end, the default is to stop the actuators.
    thriftyClimberS.setDefaultCommand(stopBackC().perpetually());
    linearClimberS.setDefaultCommand(stopFrontC().perpetually());
    tiltClimberS.setDefaultCommand(stopTiltC().perpetually());
  }

  /**
   * Unlocks the climbers, allowing them to be commanded.
   */
  public void unlock() {
    climberLocked = false;
  }

  /**
   * Locks the climbers, preventing them from being commanded.
   */
  public void lock() {
    climberLocked = true;
  }

  /**
   * @return true if the climbers are locked, false otherwise.
   */
  public boolean getIsLocked() {
    return climberLocked;
  }
  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to extend the Thrifty climbers at bar-transferring speed.
   */  
  public Command transferBackC() {
    return errorClimberC(
      new RunEndCommand(thriftyClimberS::transferBack, thriftyClimberS::stopBack, thriftyClimberS)
    );
  }
  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to extend the Thrifty climbers
   */  
  public Command extendBackC() {
    return errorClimberC(
      new RunEndCommand(thriftyClimberS::extendBack, thriftyClimberS::stopBack, thriftyClimberS)
    );
  }

  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to retract the Thrifty climbers
   */  
  public Command retractBackC() {
    return errorClimberC(new RunEndCommand(thriftyClimberS::retractBack, thriftyClimberS::stopBack, thriftyClimberS));
  }

  /**
   * NOTE: Default Command for ThriftyClimberS
   * @return the Command to stop the Thrifty Climbers
   */  
  public Command stopBackC() { // We can skip the error check if we're just stopping it.
    return new InstantCommand(thriftyClimberS::stopBack, thriftyClimberS);
  }

  // Front Linear Climber
  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to retract the linear climbers at bar-transferring speed.
   */  
  public Command transferFrontC() {
    return errorClimberC(linearClimberS.transferFrontC());
  }

    /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to extend the linear climbers
   */
  public Command extendFrontC() {
    return errorClimberC(linearClimberS.extendFrontC());
  }

  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to retract the linear climbers
   */
  public Command retractFrontC() {
    return errorClimberC(linearClimberS.retractFrontC());
  }

  /**
   * NOTE: Default Command for LinearClimberS
   * @return the Command to stop the linear climbers
   */
  public Command stopFrontC() {
    return linearClimberS.stopFrontC();
  }

  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to tilt the climber forward
   */
  public Command forwardTiltC() {
    return errorClimberC(new RunEndCommand(tiltClimberS::tiltForward, tiltClimberS::tiltStop, tiltClimberS));
  }

  /**
   * NOTE: this will not run if the climber is locked!
   * @return the Command to tilt the climber back
   */
  public Command backwardTiltC() {
    return errorClimberC(new RunEndCommand(tiltClimberS::tiltBack, tiltClimberS::tiltStop, tiltClimberS));
  }

  /**
   * NOTE: Default Command for TiltClimberS.
   * @return the Command to stop the climber tilt at its current position.
   */
  public Command stopTiltC() {
    return new InstantCommand(tiltClimberS::tiltStop, tiltClimberS);
  }
  /**
   * A utility method to create a command that runs the attempted command if the climber is not locked,
   * and flashes the lights in an error pattern otherwise.
   * 
   * NOTE: This only checks the error condition when scheduled. If the condition changes while the command is running,
   * the command will NOT be interrupted.
  */
  private Command errorClimberC(Command attemptCommand) {
    return new ConditionalCommand( // branching logic.
      // Conditional Command requires the requirements of its true and false branches
      // if condition is true
      // ScheduleCommand allows us to schedule a command 
      // that is not actually part of executing the ConditionalCommand.
      // This means that errorClimberC will end instantly if the climber is locked,
      // and the lights command will run separately for its 0.5 seconds.
      new ScheduleCommand( 
        new RunCommand(
        ()->
          LightsManager.getInstance().requestState(States.Error) // Request error states on the lights every loop...
      ).withTimeout(0.5)), // for 0.5 sec
      attemptCommand, // If condition is false, run the attempted command.
      this::getIsLocked // The condition is true if the climbers are locked.
    );
  }
}