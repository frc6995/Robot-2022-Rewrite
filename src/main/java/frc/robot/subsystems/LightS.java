// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightS extends SubsystemBase {

    //===INTERNAL CONSTANTS===

    /** The roboRIO spark value for solid green LEDs */
    public static final double LED_SOLID_GREEN = 0.77;

    /** The roboRIO spark value for Light Chase pattern */
    public static final double LED_PATTERN_GREEN = 0.01;
  
    /** The roboRIO spark value for Strobe, Red pattern */
    public static final double LED_PATTERN_RED = -0.11;
  
    /** The roboRIO spark value for Rainbow, Party Palette pattern */
    public static final double LED_PARTY_MODE = -0.97;
  
    public static final double LED_GOLD_SOLID = 0.67;
  
    public static final double LED_GREEN_RAINBOW = -0.91;
  
    public static final double LED_LARSON_SCANNER = -0.01;
  
    public static final double LED_CONFETTI_MODE = -0.87;
  
    public static final int PWM_PORT_LED = Constants.PWM_PORT_LED;


  private static LightS m_instance = new LightS();
  private Spark spark = new Spark(PWM_PORT_LED);

  /** Creates a new LightsManager. */
  private LightS() {
  }

  public static LightS getInstance() {
      return m_instance;
  }

  private TreeSet<States> m_states = new TreeSet<>();


  /**
   * Different states of the robot. Their order top-to-bottom corresponds to their priority
   * and their ordering in the TreeSet. Each state has a corresponding PWM value for the Blinkin.
   */
  public static enum States {
    Disabled(LED_SOLID_GREEN), // set in robotPeriodic
    Error(LED_PATTERN_RED),
    Climbing(LED_PARTY_MODE), // set through triggers in RobotContainer
    ShooterAndDistanceReady(LED_GOLD_SOLID), // ditto
    ShooterReady(LED_GREEN_RAINBOW), // ditto
    Shooting(LED_PATTERN_GREEN), //ditto
    Intaking(LED_PATTERN_GREEN), // set from MainCommandFactory.createIntakeCG`
    Default(LED_SOLID_GREEN);

    public final double lightSpeed;

    private States(double lightSpeed) {
      this.lightSpeed = lightSpeed;
    }
  }


  /**
   * Requests the current state of the robot, determines whether the requested
   * state is a higher priority than the current state, sets the current state to
   * the requested state
   * 
   * @param state The requested state of the robot when the method is called
   */
  public void requestState(States state) {
    m_states.add(state);
  }

  /**
   * Periodically checks the current state of the robot and sets the LEDs to the
   * corresponding light pattern
   */
  @Override
  public void periodic() {
    requestState(States.Default);
    spark.set(m_states.first().lightSpeed);
    m_states.removeAll(Set.of(States.values()));
  }

  /**
   * Returns a Command to request a state from the lights.
   * Because the lights have their own priority manager, this does NOT require the LightS.
   * @param stateSupplier
   * @return
   */
  public Command requestStateC(Supplier<States> stateSupplier) {
    return new RunCommand(() -> m_instance.requestState(stateSupplier.get()));
  }
}