// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

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

public class ThriftyClimberS extends SubsystemBase implements Loggable {

  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_BACK_CLIMB_MOTOR, MotorType.kBrushless);
  @Log(methodName = "getPosition", name = "backPosition")
  private RelativeEncoder sparkMaxEncoderTwo = backSparkMax.getEncoder();
  
  private double minimumLimit = Constants.CLIMBER_BACK_SOFT_LIMIT_SHOOTER;
  private double maximumLimit = Constants.CLIMBER_BACK_SOFT_LIMIT_FORWARD;
  /** Creates a new ClimberS. */
  public ThriftyClimberS() {

    backSparkMax.restoreFactoryDefaults();
    backSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    backSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    backSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) maximumLimit);
    backSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) minimumLimit);
    backSparkMax.setIdleMode(IdleMode.kBrake);
    backSparkMax.setSmartCurrentLimit(40, 40, 0);
    backSparkMax.burnFlash();
  }

  public double getBackPosition() {
    return sparkMaxEncoderTwo.getPosition();
  }

  public void driveBack(double voltage) {
    backSparkMax.setVoltage(voltage);
  }

  public void transferBack() {
    driveBack(Constants.CLIMBER_BACK_TRANSFER_VOLTS);
  }

  public void extendBack() {
    driveBack(10);
  }

  public void retractBack() {
    driveBack(-10);
  }

  public void stopBack() {
    driveBack(0);
  }

  public void holdBack() {
    driveBack(Constants.CLIMBER_BACK_HOLDING_FF);
  }
  
  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // if(Math.abs(backSparkMax.getAppliedOutput()) < 0.1 && backSparkMax.getEncoder().getVelocity() > 10) {
    //   holdBack();
    // }
  }
}
