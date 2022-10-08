package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.command.RunEndCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Takes balls from the intake and holds it and sends it to the shooter.
 * 
 * @authors Jonas An and Ben Su
 */
public class MidtakeS extends SubsystemBase implements Loggable{
  @Log(methodName = "getAppliedOutput", name = "frontVolts")
  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_FRONT, MotorType.kBrushless);

  @Log(methodName = "getAppliedOutput", name = "backVolts")
  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_BACK,
      MotorType.kBrushless);
  @Log(methodName = "getVelocity")
  private RelativeEncoder backEncoder = backSparkMax.getEncoder();

  private DigitalInput beamBreakTop = new DigitalInput(Constants.BEAM_BREAK_TOP_PORT_NUMBER);
  private DigitalInput beamBreakBottom = new DigitalInput(Constants.BEAM_BREAK_BOTTOM_PORT_NUMBER);
  private boolean beamBreakTopBroken = false;
  private boolean beamBreakBottomBroken = false;

  public final Trigger topBeamBreakTrigger = new Trigger(this::getIsTopBeamBroken);
  public final Trigger bottomBeamBreakTrigger = new Trigger(this::getIsBottomBeamBroken);
  public final Trigger midtakeStoppedTrigger = new Trigger(this::getIsStopped);

  /**
   * Create a new MidtakeS
   */
  public MidtakeS() {
    frontSparkMax.restoreFactoryDefaults();
    frontSparkMax.setInverted(true);
    backSparkMax.restoreFactoryDefaults();
    backSparkMax.follow(frontSparkMax, true);
    
  }


  /**
   * Sets the speed of the front motor
   * 
   * @param frontVolts the front speed
   */
  public void spin(double volts) {
    frontSparkMax.setVoltage(volts);
  }

  /**
   * Spins both midtake motors at set speed.
   */
  public void load() {
    spin(Constants.MIDTAKE_LOADING_VOLTS);
  }

  public void reverse() {
    spin(-Constants.MIDTAKE_CRAWL_VOLTS);
  }

  public void feed() {
    spin(Constants.MIDTAKE_FEEDING_VOLTS);
  }

  public void crawl() {
    spin(Constants.MIDTAKE_CRAWL_VOLTS);
  }

  public void stop() {
    spin(0);
  }
  /**
   * Returns whether the top Beam Break sensor is triggered
   *
   */
  @Log
  public boolean getIsTopBeamBroken() {
    return beamBreakTopBroken;
  }

  public boolean getIsTopBeamClear() {
    return !beamBreakTopBroken;
  }

  /**
   * Returns whether the bottom Beam Break sensor is triggered
   */
  @Log
  public boolean getIsBottomBeamBroken() {
    return beamBreakBottomBroken;
  }

  public boolean getIsBottomBeamClear() {
    return !beamBreakBottomBroken;
  }

  public boolean getIsStopped() {
    return frontSparkMax.getAppliedOutput() <= 0.05 && backSparkMax.getAppliedOutput() <= 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isDisabled()) {
      stop();
    }

    if(RobotBase.isReal()) {
      beamBreakBottomBroken = !beamBreakBottom.get();
      beamBreakTopBroken = !beamBreakTop.get();
    }
  }

  public Command loadC() {
    return new RunEndCommand(this::load, this::stop, this);
  }

  public Command reverseC() {
    return new RunEndCommand(this::reverse, this::stop, this);
  }

  public Command feedC() {
    return new RunEndCommand(this::feed, this::stop, this);
  }

  public Command crawlC() {
    return new RunEndCommand(this::crawl, this::stop, this);
  }

  public Command stopC() {
    return new InstantCommand(this::stop, this);
  }

  /*
   * If Shooting:
    Pulse Midtake @ Fast Short Bursts. (Repeatedly)
Else if intaking:
    When/While Bottom BB broken & Top BB clear, drive up
Else:
   When/While Bottom BB clear, drive down.
   */

  public Command intakeC() {
    return 
      // go straight into the repeating wait-drive-wait-drive cycle
      new RepeatCommand(
        new SequentialCommandGroup(
          // Wait until the bottom is triggered and the top is clear
          new WaitUntilCommand(
            bottomBeamBreakTrigger
            .and(
              topBeamBreakTrigger.negate()
            )),
          // Load the incoming ball until the bottom is clear or the top is broken.
          loadC()
            .until(bottomBeamBreakTrigger.negate().or(topBeamBreakTrigger))
        )
    );
  }

  public Command shootC() {
    return new SequentialCommandGroup(
    feedC().withTimeout(0.2),
          new WaitCommand(1.25),
      new RepeatCommand(
        new SequentialCommandGroup(
          feedC().withTimeout(0.25),
          new WaitCommand(1.25)
        )
      ));
  }

  public Command idleC() {
    return 
      new RepeatCommand(
        reverseC().until(bottomBeamBreakTrigger)
      );
  }
}