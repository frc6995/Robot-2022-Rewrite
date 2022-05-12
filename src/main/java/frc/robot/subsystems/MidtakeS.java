package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.color.PicoColorSensor;
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
    spin(-Constants.MIDTAKE_LOADING_VOLTS);
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

  public Command createLoadC() {
    return new RunEndCommand(this::load, this::stop, this);
  }

  public Command createReverseC() {
    return new RunEndCommand(this::reverse, this::stop, this);
  }

  public Command createFeedC() {
    return new RunEndCommand(this::feed, this::stop, this);
  }

  public Command createCrawlC() {
    return new RunEndCommand(this::crawl, this::stop, this);
  }

  public Command createStopC() {
    return new InstantCommand(this::stop, this);
  }

  public Command createLowIndexC() {
    return new RepeatCommand(
      new SequentialCommandGroup(
        new WaitUntilCommand(bottomBeamBreakTrigger.and(topBeamBreakTrigger.negate())),
        createLoadC()
          .withInterrupt(bottomBeamBreakTrigger.negate().or(topBeamBreakTrigger))
      )
    );
  }

  public Command createReadyIntakeC() {
    return new SequentialCommandGroup(
      createReverseC()
        .withTimeout(0.5)
        .withInterrupt(bottomBeamBreakTrigger),
      createCrawlC()
        .withTimeout(0.25)
        .withInterrupt(bottomBeamBreakTrigger.negate())
    );
  }

  public Command createBringToTopC() {
    return new ConditionalCommand(
        new InstantCommand(),
        createLoadC()
          .withInterrupt(this::getIsTopBeamBroken),
        this::getIsTopBeamBroken);
  }

  public Command createDefaultC() {
    return new RepeatCommand(createBringToTopC());
  }

  /**
   * Creates a command to fire one ball. 
   * 
   * Assumes the ball is already at the top. 
   * @return
   */
  public Command createFeedOneC() {
    return new SequentialCommandGroup(
      new ConditionalCommand( // First thing, check if there's a ball blocking the top sensor
        new InstantCommand(), // if so, skip this part
        createFeedC() // else, feed for 0.2 seconds or until the sensor is blocked, then stop
          .withTimeout(0.2)
          .withInterrupt(topBeamBreakTrigger),
        topBeamBreakTrigger
      ),
      new ConditionalCommand( // Check if the sensor is actually blocked now
        new SequentialCommandGroup( // if so, do the following in sequence
          createFeedC().withTimeout(0.1), // Quick-burst the midtake to feed one ball
          createReverseC() // Quick-reverse to reset. Stop reversing if the top beam is clear
            .withInterrupt(topBeamBreakTrigger.negate())
            .withTimeout(0.25)
        ),
        new InstantCommand(),
        topBeamBreakTrigger
      ), // The reversal handles the cases when a second ball goes too far up when firing.
      createCrawlC() // finally, feed for 0.2 seconds or until the sensor is blocked, then stop
        .withTimeout(0.2)
        .withInterrupt(topBeamBreakTrigger)
    );
  }


}