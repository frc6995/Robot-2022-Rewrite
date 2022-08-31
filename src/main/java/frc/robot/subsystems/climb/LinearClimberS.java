// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.command.RunEndCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LinearClimberS extends SubsystemBase implements Loggable {
  
  /**
   * The forward soft limit extending the climber.
   * This is a float because of the Spark Max API.
   */
  public static final float SOFT_LIMIT_FORWARD_REVS = 200.0f;
  /**
   * The reverse soft limit for the climber (stopping above the shooter).
   * This is a float because of the Spark Max API.
   */
  public static final float SOFT_LIMIT_MID_REVS = 55.0f;
  /**
   * The reverse soft limit for the climber (stopping almost at the hard stop).
   * This is a float because of the Spark Max API.
   */
  public static final float SOFT_LIMIT_REVERSE_REVS = 14.0f;
  /**
   * The voltage applied when transferring.
   */
  public static final double TRANSFER_VOLTS = -2;
  /**
   * The voltage applied when extending.
   */
  public static final double EXTEND_VOLTS = 10;
  /**
   * The voltage applied when retracting.
   */
  public static final double RETRACT_VOLTS = -7.5;

  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_CLIMBER_MOTOR, MotorType.kBrushless);
  @Log(methodName = "getPosition", name = "frontPosition")
  private RelativeEncoder sparkMaxEncoder = frontSparkMax.getEncoder();

  /** Creates a new ClimberS. */
  LinearClimberS() {
    
    frontSparkMax.restoreFactoryDefaults();
    frontSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    frontSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    frontSparkMax.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_FORWARD_REVS);
    frontSparkMax.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_MID_REVS);
    brake();
    frontSparkMax.setSmartCurrentLimit(40, 40, 0);
    frontSparkMax.burnFlash();
    setDefaultCommand(stopFrontC());
  }

  /** @return the climber position in motor rotations */
  public double getFrontPosition() {
    return sparkMaxEncoder.getPosition();
  }

  /**
   * Applies the specified voltage to the motor.
   * @param voltage the voltage
   */
  public void driveFront(double voltage) {
    frontSparkMax.setVoltage(voltage);
  }
  /**
   * 
   * @param voltage the DoubleSupplier providing the voltage
   * @return
   */
  public Command driveFrontC(DoubleSupplier voltage) {
    return new RunEndCommand(()->driveFront(voltage.getAsDouble()), this::stopFront, this);
  }

  /** Retracts the linear climbers at bar-transferring speed. */
  public void transferFront() {
    driveFront(TRANSFER_VOLTS);
  }

  /** @return the Command to retract the linear climbers at bar-transferring speed. */  
  public Command transferFrontC() {
    return driveFrontC(()->TRANSFER_VOLTS);
  }

  /** Extends the linear climbers. */
  public void extendFront() {
    driveFront(EXTEND_VOLTS);
  }

  /** @return the Command to extend the linear climbers. */  
  public Command extendFrontC() {
    return driveFrontC(()->EXTEND_VOLTS);
  }

  /** Retracts the linear climbers. */
  public void retractFront() {
    driveFront(RETRACT_VOLTS);
  }

  /** @return the Command to retract the linear climbers. */
  public Command retractFrontC() {
    return driveFrontC(()->RETRACT_VOLTS);
  }

  /** Stops the linear climbers */
  public void stopFront() {
    driveFront(0);
  }

  /** @return the Command to stop the linear climbers. */
  public Command stopFrontC() {
    return new InstantCommand(this::stopFront, this);
  }

  /**
   * Resets the Spark MAX encoder.
   * NOTE: Only do this when actually at the lower hard limit. This will change where the soft limits fall otherwise.
   */
  public void reset() {
    sparkMaxEncoder.setPosition(0);
  }

  /** Sets the motor to brake mode. */
  public void brake() {
    frontSparkMax.setIdleMode(IdleMode.kBrake);
  }
  /** Sets the motor to coast mode. */
  public void coast() {
    frontSparkMax.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
  }
}
