package frc.robot.subsystems;

import static frc.robot.Constants.CAN_ID_BACK_SHOOTER_MOTOR;
import static frc.robot.Constants.CAN_ID_FRONT_SHOOTER_MOTOR;
import static frc.robot.Constants.SHOOTER_BACK_FF;
import static frc.robot.Constants.SHOOTER_FRONT_FF;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.LightS.States;
import frc.robot.util.SimEncoder;
import frc.robot.util.command.RunEndCommand;
import frc.robot.util.interpolation.ShooterInterpolatingTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The shooter subsystem.
 * This handles the two sets of wheels that shoot the cargo into the hub
 * 
 * @author Noah Kim
 */
public class ShooterS extends SubsystemBase implements Loggable {
  @Log(methodName = "getAppliedOutput")
  private final CANSparkMax frontSparkMax = new CANSparkMax(CAN_ID_FRONT_SHOOTER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backSparkMax = new CANSparkMax(CAN_ID_BACK_SHOOTER_MOTOR, MotorType.kBrushless);
  private Encoder frontEncoder;
  private Encoder backEncoder;

  // flywheel rotations per second
  private SimpleMotorFeedforward frontFF = new SimpleMotorFeedforward(
    SHOOTER_FRONT_FF[0],
    SHOOTER_FRONT_FF[1],
    SHOOTER_FRONT_FF[2]);
  private FlywheelSim frontSim = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(
      Units.radiansToRotations(SHOOTER_FRONT_FF[1]), 
      Units.radiansToRotations(SHOOTER_FRONT_FF[2])), 
      DCMotor.getNEO(1), 1);


  private SimpleMotorFeedforward backFF = new SimpleMotorFeedforward(
    SHOOTER_BACK_FF[0],
    SHOOTER_BACK_FF[1],
    SHOOTER_BACK_FF[2]);
    private PIDController frontPID = new PIDController(Constants.SHOOTER_FRONT_P, 0,0);
  private PIDController backPID = new PIDController(Constants.SHOOTER_BACK_P, 0,0);
  private SimEncoder frontSimEncoder = new SimEncoder();
  private SimEncoder backSimEncoder = new SimEncoder();
    private FlywheelSim backSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(
        Units.radiansToRotations(SHOOTER_BACK_FF[1]), 
        Units.radiansToRotations(SHOOTER_BACK_FF[2])), 
        DCMotor.getNEO(1), 1);
  private double frontSetpointRPS = 0;
  private double backSetpointRPS = 0;
  public final Trigger atTargetTrigger = new Trigger(this::isAtTarget);
  
  /** Creates a new ShooterS. */
  public ShooterS() {
    frontSparkMax.restoreFactoryDefaults();
    backSparkMax.restoreFactoryDefaults();
    frontSparkMax.setSmartCurrentLimit(20, 30, 0);
    backSparkMax.setSmartCurrentLimit(20, 30, 0);
    
    frontEncoder = new Encoder(Constants.SHOOTER_FRONT_ENCODER[0], Constants.SHOOTER_FRONT_ENCODER[1]);
    frontEncoder.setDistancePerPulse(1.0/Constants.SHOOTER_ENCODER_CPR);
    backEncoder = new Encoder(Constants.SHOOTER_BACK_ENCODER[0], Constants.SHOOTER_BACK_ENCODER[1]);
    backEncoder.setDistancePerPulse(1.0/Constants.SHOOTER_ENCODER_CPR);
    backEncoder.setSamplesToAverage(3);
  }

  /**
   * Gets the front encoder speed
   * 
   * @return The velocity of the front motor in RPM
   */
  @Log
  public double getFrontEncoderSpeed() {
    if(RobotBase.isReal()) {
      return frontEncoder.getRate() * 60.0; //rate is in rot/s so multiply by 60 for rpm
    } else {
      return frontSimEncoder.getVelocity();
    }
  }

  /**
   * Gets the back encoder speed
   * 
   * @return The velocity of the back motor
   */
  @Log
  public double getBackEncoderSpeed() {
    if (RobotBase.isReal()) {
      return backEncoder.getRate() * 60.0;
    }
    else {
      return backSimEncoder.getVelocity();
    }
  }

  /**
   * Sets the voltage of the front motor
   * 
   * @param speed Voltage value for the front motor
   */
  public void setFrontVolts(double speed) {
    frontSparkMax.setVoltage(speed);
  }

  /**
   * Sets the voltage of the back motor
   * 
   * @param speed Voltage value for the back motor
   */
  public void setBackVolts(double speed) {
    backSparkMax.setVoltage(speed);
  }

  /**
   * Sets the speed of the front motor using FF in RPM
   * 
   * @param frontTargetRPM The target RPM of the front motor
   */
  public void setFrontVelocity(double frontTargetRPM) {
    frontSetpointRPS = frontTargetRPM / 60.0;
    setFrontVolts(frontFF.calculate(frontSetpointRPS) + frontPID.calculate(
      getFrontEncoderSpeed() / 60.0, frontSetpointRPS));

  }

  /**
   * Sets the speed of the back motor using FF in RPM
   * 
   * @param backTargetRPM The target RPM of the front motor
   */
  public void setBackVelocity(double backTargetRPM) {
    backSetpointRPS = backTargetRPM / 60.0;
    setBackVolts(backFF.calculate(backSetpointRPS) + 
      backPID.calculate(
        getBackEncoderSpeed() / 60.0, backSetpointRPS));
  }

  /**
   * stops both motors
   */
  public void stop() {
    setFrontVolts(0);
    setBackVolts(0);
  }

  /**
   * determines whether the front motor is at the target RPM
   * 
   * @return True if the front motor is at the target RPM
   */
  @Log
  public boolean isFrontAtTarget() {
    return  frontPID.getVelocityError() < Constants.SHOOTER_PID_ERROR;
  }

  /**
   * determines whether the back motor as at the target RPM
   * 
   * @return True if the back motor is at the target RPM
   */
  @Log
  public boolean isBackAtTarget() {
    return backPID.getVelocityError() < Constants.SHOOTER_PID_ERROR;
  }

  /**
   * determines whether both motors are at the target RPMs.
   * 
   * @return True if both motors are at the target RPMs
   */
  @Log
  public boolean isAtTarget() {
    return isBackAtTarget() && isFrontAtTarget();
  }

  @Override
  public void periodic() {
    if(isMoving()) {
      LightS.getInstance().requestState(States.Shooting);
    }
  }

  public boolean isMoving() {
    return getFrontEncoderSpeed() > 100 && getBackEncoderSpeed() > 100;
  }

  public void simSpeedDrop(){
    Matrix<N1, N1> newFrontState = new Matrix<N1, N1>(Nat.N1(), Nat.N1()); 
    newFrontState.set(0, 0, Units.rotationsPerMinuteToRadiansPerSecond(getFrontEncoderSpeed() - 500));
    Matrix<N1, N1> newBackState = new Matrix<N1, N1>(Nat.N1(), Nat.N1()); 
    newBackState.set(0, 0, Units.rotationsPerMinuteToRadiansPerSecond(getBackEncoderSpeed() - 500));
    frontSim.setState(newFrontState);
    backSim.setState(newBackState);
  }

  @Override
  public void simulationPeriodic() {
    frontSim.setInput(frontSparkMax.getAppliedOutput() - 
      (Math.signum(frontSparkMax.getAppliedOutput()) * SHOOTER_FRONT_FF[0]));
    backSim.setInput(backSparkMax.getAppliedOutput() - 
      (Math.signum(backSparkMax.getAppliedOutput()) * SHOOTER_BACK_FF[0]));
    frontSim.update(0.02);
    backSim.update(0.02);

    frontSimEncoder.setVelocity(frontSim.getAngularVelocityRPM());
    backSimEncoder.setVelocity(backSim.getAngularVelocityRPM());
  }

  public Command spinVelocityC(DoubleSupplier frontVelocityRPM, DoubleSupplier backVelocityRPM) {
    return new RunEndCommand(
      ()->{
        this.setFrontVelocity(
          frontVelocityRPM.getAsDouble()
        );
        this.setBackVelocity(
          backVelocityRPM.getAsDouble()
        );
      }, this::stop, this);
  }

  public Command spinDistanceC(DoubleSupplier distance) {
    return spinVelocityC(
      () -> ShooterInterpolatingTable.get(distance.getAsDouble()).frontWheelRpm,
      () -> ShooterInterpolatingTable.get(distance.getAsDouble()).backWheelRpm
    );
  }

  public Command stopC() {
    return new InstantCommand(this::stop, this);
  }
}
