package frc.robot.subsystems;

import org.photonvision.PhotonVersion;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * @author Valentine King
 */

public class Limelight implements Loggable {

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

  /** Creates a new LimelightS. */
  public Limelight() {
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setString(
        PhotonVersion.versionString);
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
  }

  public void ledsOn() {
    setLED(true);
  }

  public void ledsOff() {
    setLED(false);
  }

  public boolean hasTarget() {
    return false;
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
   * double distance = NomadMathUtil.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(targetY),
            Units.degreesToRadians(targetX)
          ) + Constants.HUB_RADIUS_METERS;
   */
  public void periodic() {

  }
}
