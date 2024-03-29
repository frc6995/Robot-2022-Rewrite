package frc.robot.subsystems;

import static frc.robot.Constants.CAN_ID_TURRET;
import static frc.robot.Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION;
import static frc.robot.Constants.SOFT_LIMIT_FORWARD_RADIAN;
import static frc.robot.Constants.SOFT_LIMIT_REVERSE_RADIAN;
import static frc.robot.Constants.TURRET_D;
import static frc.robot.Constants.TURRET_FF;
import static frc.robot.Constants.TURRET_MAX_SPEED;
import static frc.robot.Constants.TURRET_P;
import static frc.robot.Constants.TURRET_PID_ERROR;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.SimEncoder;
import frc.robot.util.command.RunEndCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The turret subsystem. It uses a PID controller to control the turret position.
 * 
 * CCW POSITIVE
 * 
 * @author Benjamin Su
 * @author Noah Kim
 */
public class TurretS extends SubsystemBase implements Loggable {
  private CANSparkMax sparkMax = new CANSparkMax(CAN_ID_TURRET, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder();

  /**
   * Set up a profiled pid controller.
   * 
   * This controller controls the position of the turret to reach a given position goal. 
   * It outputs a desired velocity for the turret.
   * 
   * In order to avoid huge "panic mode" velocity commands when the goal changes suddenly to a different value,
   * the actual position setpoint that is being tracked is limited by a motion profile,
   * which limits how quickly it can change (max setpoint velocity)
   * and how quickly that velocity can change (max setpoint acceleration).
   * This smooths out the motion to the new goal.
   * 
   * 
   */
  private ProfiledPIDController turretPID = new ProfiledPIDController(TURRET_P, 0, TURRET_D, new Constraints(2*Math.PI, 6*Math.PI));

  /**
   * Set up a SimEncoder to store the position and velocity of the mechanism as output by the simulator. 
   * 
   * For various reasons relating to REV's sim support, we can't directly tell the simulated SparkMaxEncoder what to report.
   */
  private SimEncoder turretSimEncoder = new SimEncoder();
  // Open-loop drive in turret radians per second
  /**
   * Set up the velocity to voltage FF formula
   * SimpleMotorFeedforward(
   *  kS (V),
   *  kV (V/(rad/s)),
   *  kA (V/(rad/s^2))
   * )
   * 
   * Voltage = kS + kV * vel + kA * accel
   * 
   * (We don't actually use the accel term here, but if the mechanism had a lot of inertia,
   * we'd want to specify an additional voltage boost if we expect the velocity to keep increasing.
   * In that case we would essentially instruct the mechanism to overshoot the specified velocity slightly
   * to keep up with accel)
   */
  private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(

    TURRET_FF[0],
    TURRET_FF[1],
    TURRET_FF[2]);

  // Set up a sim to estimate the position response of a motor with the given FF constants.
  private LinearSystemSim<N2, N1, N1> turretSim = new LinearSystemSim<N2, N1, N1>(
    LinearSystemId.identifyPositionSystem(TURRET_FF[1], TURRET_FF[2])
    );

  NetworkButton turretResetNetworkButton;

  /** Creates a new TurretS. */
  public TurretS() {
    sparkMax.restoreFactoryDefaults();
    // Automatically multiply NEO rotations to read encoder in turret radians.
    // Position from NEO rotations to turret radians: 2pi turret radians / 1 turret revolution * 1 turret revolution/
    sparkMaxEncoder.setPositionConversionFactor(2 * Math.PI / NEO_REVOLUTIONS_PER_TURRET_REVOLUTION);
    sparkMaxEncoder.setVelocityConversionFactor(2 * Math.PI / NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 60);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_FORWARD_RADIAN);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) SOFT_LIMIT_REVERSE_RADIAN);
    turretPID.setTolerance(TURRET_PID_ERROR, Units.degreesToRadians(1.5)); 
    // 1.5 deg/s allows us to shut off the PID and cut voltage
    // even if the encoder is still changing a bit. Without that, even the tiniest oscillation means the pid is not at its goal,
    // even if the position is within tolerance.
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setSmartCurrentLimit(20, 20, 0);
    
    // When starting up, we want the encoder to read pi radians (180 deg) offset from the "forward" direction of the bot.
    resetEncoder();
    // We want the PID controller to default to holding that starting position.
    turretPID.setGoal(getEncoderCounts());

    // Set up logging that can't easily be done with the @Log annotation
    Shuffleboard.getTab("TurretS").addNumber("Tgt", ()->turretPID.getGoal().position);
    Shuffleboard.getTab("TurretS").addNumber("Setpos", ()->turretPID.getSetpoint().position);
    Shuffleboard.getTab("TurretS").addNumber("Setvel", ()->turretPID.getSetpoint().velocity);
    /**
     * Set up an encoder reset bound to a Network Tables boolean value.
     */
    Shuffleboard.getTab("TurretS")
    .add("requestTurrReset", false) //Add the boolean to Shuffleboard.
    .getEntry()
    .addListener((notif)->{ // Add a listener for changes to the value.
      if(notif.value.getBoolean()) { // If the value changes to true, 
        resetEncoder(); // reset encoder
      }
    }, 0);


  }

  /**
   * Gets encoder counts from spark max. Uses the sim encoder if in simulation.
   * 
   * @return the encoder counts
   */
  @Log(name = "turretPosition")
  public double getEncoderCounts() {
      if(RobotBase.isReal()) {
        return sparkMaxEncoder.getPosition();
      }
      else {
        return turretSimEncoder.getPosition();
      }
  }

  /**
   * Gets the rotation offset of the turret from the robot front as a Rotation2d, useful for geometry math.
   * @return
   */
  public Rotation2d getRobotToTurretRotation() {
    return new Rotation2d(getEncoderCounts());
  }
  
  /**
   * Sets the turn speed of the turret as a fraction of the max allowed speed.
   * 
   * @param speed The turn speed of the turret [-1..1]
   */
  public void setSpeed(double speed) {
    setVelocity(speed * TURRET_MAX_SPEED);
  }

  /**
   * Stops the motor.
   */
  public void stopMotor() {
    sparkMax.setVoltage(0);
  }

  /**
   * Resets encoder count to Pi radians.
   */
  public void resetEncoder() {
    resetEncoder(Math.PI);
  }

  /**
   * Resets encoder count.
   */
  public void resetEncoder(double radians) {
    if(RobotBase.isReal()) {
      sparkMaxEncoder.setPosition(radians);
    }
    else {
      turretSimEncoder.setPosition(radians);
      // If we don't tell the sim the position is changing without the motor moving, 
      // velocity will be correct, but position will still be reported from the old value.
      setSimState(radians, getVelocity());
    }
  }

  /**
   * Sets the current position and velocity in the simulated system. 
   * @param position
   * @param velocity
   */
  private void setSimState(double position, double velocity) {
    // The simulator requests a 2x1 grid of position and velocity. We use a matrix library to fill that grid
    Matrix<N2, N1> newState = new Matrix<N2, N1>(Nat.N2(), Nat.N1()); 
    newState.set(0, 0, position);
    newState.set(1, 0, velocity);
    turretSim.setState(newState);
  }

  /**
   * Commands the FeedForward to turn the motor at the specified velocity
   * @param velocity The target velocity in turret radians per second.
   */
  private void setVelocity(double velocity) {
    setVoltage(turretFF.calculate(velocity));
  }


  /**
   * Sets raw voltage to the turret motor.
   * @param voltage
   */
  private void setVoltage(double voltage) {
    sparkMax.setVoltage(voltage);
  }

  /**
   * Checks if a given position target is within the soft limits
   * @param target
   */
  public boolean isTargetInRange (double target) {
    double targetRadians = NomadMathUtil.modulus(target);

    if(targetRadians > Constants.SOFT_LIMIT_FORWARD_RADIAN) {
      return false;
    }
    else if(targetRadians < Constants.SOFT_LIMIT_REVERSE_RADIAN) {
      return false;
    }
    return true;
  }

  /**
   * Checks if a given position target is within the soft limits
   * @param target
   */
  public boolean isTargetInRange (Rotation2d target) {
    double targetRadians = NomadMathUtil.modulus(target);

    if(targetRadians > Constants.SOFT_LIMIT_FORWARD_RADIAN) {
      return false;
    }
    else if(targetRadians < Constants.SOFT_LIMIT_REVERSE_RADIAN) {
      return false;
    }
    return true;
  }


  /**
   * Gets the velocity of the NEO motor.
   * 
   * @return The velocity of the motor, in turret radians per second
   */
  @Log(name = "turretVelocity")
  public double getVelocity() {
    if(RobotBase.isReal()) {
      return sparkMaxEncoder.getVelocity();
    }
    else {
      return turretSimEncoder.getVelocity();
    }
  }

  /**
   * Sets the goal of the PID controller and commands the velocity the controller outputs.
   * @param target The desired angle, relative to the +x axis of the robot coordinate frame.
   */
  public void setTurretAngle(Rotation2d target) {
    // the given target switches from -pi to pi as it crosses the back of the robot,
    // So we add 2pi to the negative half, giving us an equivalent range of 0 to 2pi,
    // and no big jump right in our operating range
    double targetPosition = NomadMathUtil.modulus(target);
    // Set the new goal of the PID to this adjusted target.
    turretPID.setGoal(targetPosition);

    
    // If we are within the position and velocity acceptable range,
    if(turretPID.atGoal()) {
      setVoltage(0); // Just shut off the motor so we don't get micro-oscillations.
    }
    else {
      // Otherwise, run the PID controller and command the output velocity.
      // PID calculates velocity needed to minimize position error between reality and the (moving) setpoint
      double pidVelocity = turretPID.calculate(getEncoderCounts());
      // We add the rate at which the profiled position setpoint is changing, as somewhat of a feedforward.
      setVelocity(pidVelocity + turretPID.getSetpoint().velocity);
      // The PID is tuned quite low because the feedforward does most of the work of tracking the setpoint's velocity.
    }
  }

  @Override
  public void simulationPeriodic() {
    // Get the voltage we actually tried to send to the motor controller.
    double voltage = sparkMax.getAppliedOutput();

    // simulate the soft-limits' constraints on voltage.

    if (getEncoderCounts() < SOFT_LIMIT_REVERSE_RADIAN) { // If we are past the reverse limit (low-end position, negative velocity)
      voltage = Math.max(0, voltage); // Voltage pushing more reverse should be clipped to 0. 
      // 0 is greater than this voltage in that case, so we use the maximum of 0 and the voltage.
    } else if (getEncoderCounts() > SOFT_LIMIT_FORWARD_RADIAN) { // If we are past the forward limit (high-end position, positive velocity)
      voltage = Math.min(-0, voltage); // Voltage pushing more forward should be clipped to 0. 
      // 0 is less than this voltage in that case, so we use the minimum of 0 and the voltage.
    }

    // Send it to the sim motor if we are enabled, If we are disabled, send 0 voltage to the sim motor.
    if(DriverStation.isEnabled()) {
        // Subtract the kS term because the simulation has no static friction,
        // so we don't want to be supplying the minimum voltage we need to overcome it IRL.
        // If we don't do this, if we command 0, it will actually supply kS volts (~0.15 V),
        // Which the sim will consider enough to make the turret move.
        turretSim.setInput(NomadMathUtil.subtractkS(voltage, TURRET_FF[0]));
    }
    else {
      turretSim.setInput(0);
    }

    // Calculate the physics simulation over the next loop time (nominally 0.02 sec)
    turretSim.update(0.02);
    // Row 0, column 0 is position (in the position units of our FF constants, radians)
    double newPosition = turretSim.getOutput().get(0, 0);

    // Position delta velocity calculation. Velocity is distance over time,
    // so we measure the change in position over the last 0.02 s.
    double newVelocity = (newPosition - getEncoderCounts()) / 0.02;

    turretSimEncoder.setPosition(newPosition); //Set the simulated encoder that the rest of the subsystem reads
    // so it can work off of the adjusted value for the next update.
    turretSimEncoder.setVelocity(newVelocity);
  }

  /**
   * Returns the radian error that when added to the current position, would give the target position. 
   * @param target
   * @return
   */
  public double getError(Rotation2d target) {
    return target.minus(getRobotToTurretRotation()).getRadians();
  }

  /**
   * determines whether the turret is at the target angle
   * 
   * @return True if the turret is at the target angle
   */
  public boolean isAtTarget(Rotation2d target) {
    return Math.abs(getError(target)) < Constants.TURRET_PID_ERROR;
  }

  /**
   * Resets the PID controller. Called before the controller begins being used.
   * 
   * The controller requires that `calculate()` be called once every loop 
   * in order to get correct data for the derivative term. By resetting
   * just before using the values, we can correct for a long gap without calculating 
   * (e.g. when in manual control)
   */
  public void resetPID() {
    turretPID.reset(getEncoderCounts(), getVelocity());
  }
  
  @Override
  public void periodic() {
  }


  /* COMMANDS
    Though an argument can be made that the subsystem should not manage its own commands, we expose the subsystem's
    actions as simple commands here.
  */

  public Command manualC(DoubleSupplier speed) {
    return new RunEndCommand(()->this.setSpeed(speed.getAsDouble()), this::resetPID, this);
  }

  /**
   * Creates a command that drives the turret to the angle supplied by the given DoubleSupplier
   * @param angle
   * @return
   */
  public Command turnAngleC(DoubleSupplier angle) {
    return (
      new RunCommand(()->this.setTurretAngle(new Rotation2d(angle.getAsDouble())), this)
    );
  }

  /**
   * Creates a command that minimizes the error externally supplied through the given DoubleSupplier. Use this for Limelight.
   * 
   * 
   * @param error The error in radians. Positive error means the turret should turn CCW
   * @return
   */
  public Command zeroErrorC(DoubleSupplier error) {
    return manualC(()->{return Constants.TURRET_P * error.getAsDouble();});
  }

  /**
   * Creates a command that minimizes the error externally supplied through the given DoubleSupplier.
   * Also includes a manual override speed input. If this speed is greater than 0, it will be used in place of the error calc.
   * 
   * 
   * @param limelightOffset A Supplier for the error in radians. Positive error means the turret should turn CCW
   * @param manualSpeed A Supplier for a manual override speed. (Positive means CCW)
   *  
   * @return
   */
  public Command aimWithLimelight(DoubleSupplier limelightOffset, DoubleSupplier manualSpeed) {
    return new RunEndCommand(()->{
      if (Math.abs(manualSpeed.getAsDouble()) > 0.05) {
        this.setSpeed(manualSpeed.getAsDouble());
      }
      else {
        this.setVoltage(turretFF.calculate((-limelightOffset.getAsDouble() * 6)));
      }
    }
    , this::resetPID, this);
  }
}