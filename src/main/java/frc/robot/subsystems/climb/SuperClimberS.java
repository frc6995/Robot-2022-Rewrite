// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsManager;
import frc.robot.subsystems.LightsManager.States;
import frc.robot.util.command.RunEndCommand;
import io.github.oblarg.oblog.Loggable;

import static frc.robot.util.command.CommandUtil.errorC;

public class SuperClimberS implements Loggable {

  public final ThriftyClimberS thriftyClimberS = new ThriftyClimberS();
  public final LinearClimberS linearClimberS = new LinearClimberS();
  public final TiltClimberS tiltClimberS = new TiltClimberS();

  private boolean locked = true;

  public final Trigger climberLockedTrigger = new Trigger(this::getIsLocked);
  
  /** Creates a new ClimberS. */
  public SuperClimberS() {
  }

  public void unlock() {
    locked = false;
  }

  public void lock() {
    locked = true;
  }

  public boolean getIsLocked() {
    return locked;
  }

  public Command transferBackC() {
    return errorClimberC(
      new RunEndCommand(thriftyClimberS::transferBack, thriftyClimberS::stopBack, thriftyClimberS)
    );
  }

  public Command extendBackC() {
    return errorClimberC(
      new RunEndCommand(thriftyClimberS::extendBack, thriftyClimberS::stopBack, thriftyClimberS)
    );
  }

  public Command retractBackC() {
    return new RunEndCommand(thriftyClimberS::retractBack, thriftyClimberS::stopBack, thriftyClimberS);
  }

  public Command stopBackC() { // We can skip the error check if we're just stopping it.
    return new InstantCommand(thriftyClimberS::stopBack, thriftyClimberS);
  }

  // Front Linear Climber  
  public Command transferFrontC() {
    return errorClimberC(
      new RunEndCommand(linearClimberS::transferFront, linearClimberS::stopFront, linearClimberS)
    );
  }

  public Command extendFrontC() {
    return errorClimberC(
      new RunEndCommand(linearClimberS::extendFront, linearClimberS::stopFront, linearClimberS)
    );
  }

  public Command retractFrontC() {
    return new RunEndCommand(linearClimberS::retractFront, linearClimberS::stopFront, linearClimberS);
  }
  public Command stopFrontC() {
    return new InstantCommand(linearClimberS::stopFront, linearClimberS);
  }

  public Command tiltForwardC() {
    return errorClimberC(new RunEndCommand(tiltClimberS::tiltForward, tiltClimberS::tiltStop, tiltClimberS));
  }

  public Command tiltBackC() {
    return errorClimberC(new RunEndCommand(tiltClimberS::tiltBack, tiltClimberS::tiltStop, tiltClimberS));
  }

  public Command tiltStopC() {
    return new InstantCommand(tiltClimberS::tiltStop, tiltClimberS);
  }

  private Command errorClimberC(Command attemptCommand) {
    return errorC(
      attemptCommand,
      new RunCommand(
        ()->
          LightsManager.getInstance().requestState(States.Error)
      ).withTimeout(0.5),
      this::getIsLocked
    );
  }
}