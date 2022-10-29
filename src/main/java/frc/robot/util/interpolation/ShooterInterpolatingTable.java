package frc.robot.util.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import frc.robot.Constants;

import static java.util.Map.entry;

// Courtesy of 5940

// Interpolating table
public class ShooterInterpolatingTable {

  /* Private constructor because this is a utility class */
  private ShooterInterpolatingTable() {
  }

  public static final double MIN_DISTANCE = 3;
  public static final double MAX_DISTANCE = 4.66;

  // Interpolating tree map
  private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(Map.ofEntries(
      
      entry(2.74 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(3100, 2300)),
      entry(3.4 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(3300, 2400)),
      entry(3.74 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(3400, 2500)),
      entry(4.66 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(3400, 3100))//,
      //entry(6 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(3500, 3500))
      // 2.74 -3300, 2400
      // 3.1 - 3400, 2500
      //5 - 3500

  ));
      
  // entry(5.4 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1900, 1900,
  // 1.0))));

  // Method to get shot parameters based on vision distances
  public static ShotParameter get(double distance) {
    Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
    Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
    if (ceilEntry == null)
      return floorEntry.getValue();
    if (floorEntry == null)
      return ceilEntry.getValue();
    if (ceilEntry.getValue().equals(floorEntry.getValue()))
      return ceilEntry.getValue();
    return ceilEntry.getValue().interpolate(
        floorEntry.getValue(),
        (distance - floorEntry.getKey()) / (ceilEntry.getKey() - floorEntry.getKey()));
  }

  public static boolean getInRange(double distance) {
    return distance > MIN_DISTANCE && distance < MAX_DISTANCE;
  }
}