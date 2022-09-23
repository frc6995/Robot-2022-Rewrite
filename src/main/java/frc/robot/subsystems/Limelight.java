package frc.robot.subsystems;

import org.photonvision.PhotonVersion;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NomadMathUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * @author Valentine King
 */

public class Limelight extends SubsystemBase implements Loggable {

  // Sim stuff

  LinearFilter xOffsetFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT,
      Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);
  LinearFilter distanceFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT,
      Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);
  @Log
  private double filteredXOffsetRadians = 0;
  @Log
  private double filteredDistanceMeters = 0;
  private double lastValidDistance = 0;
  private NetworkTableEntry txEntry;
  private NetworkTableEntry tyEntry;
  private NetworkTableEntry tvEntry;
  private NetworkTableEntry ledModeEntry;

  /** Creates a new LimelightS. */
  public Limelight() {
    txEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    tyEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    tvEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    ledModeEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
  }

  /**
   * Sets the driver mode on the camera.
   * 
   * @param driverMode True to enable driver mode, false to disable driver mode.
   */
  public void setDriverMode(boolean driverMode) {

  }

  /**
   * Turns on or off the LEDs.
   * 
   * @param LED True for on, false for off.
   */
  public void setLED(boolean LED) {
    if (LED) {
      ledModeEntry.setNumber(0);
    }
    else {
      ledModeEntry.setNumber(1);
    }

  }

  public void ledsOn() {
    setLED(true);
  }

  public void ledsOff() {
    setLED(false);
  }

  public boolean hasTarget() {
    int tv = tvEntry.getNumber(0).intValue();
    return (tv != 0);
  }

  @Log
  public double getFilteredXOffset() {
    return filteredXOffsetRadians;
  }

  @Log
  public double getFilteredDistance() {
    return filteredDistanceMeters;
  }

  /**
   * 1. Gets the Limelight x and y offsets
   * 2. Calculates the distance to the target using NomadMathUtil.
   * 
   * 
   */
  @Override
  public void periodic() {
    double targetY = tyEntry.getNumber(0).doubleValue();
    double targetX = -txEntry.getNumber(0).doubleValue(); // flip to CCW positive


    double distance = NomadMathUtil.calculateDistanceToTargetMeters(
        Constants.CAMERA_HEIGHT_METERS,
        Constants.TARGET_HEIGHT_METERS,
        Constants.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(targetY),
        Units.degreesToRadians(targetX)
      ) + Constants.HUB_RADIUS_METERS;
    
      filteredDistanceMeters = distanceFilter.calculate(distance);
      filteredXOffsetRadians = xOffsetFilter.calculate(Units.degreesToRadians(targetX));
  }
}
