package frc.robot.util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NomadMathUtil {

    public static Rotation2d getDirection(Transform2d transform) {
        return getDirection(transform.getTranslation());
    }

    public static Rotation2d getDirection(Translation2d transform) {
        //add tiny number so that 0/0 comes out to 0 angle, not a div by 0 error
        return new Rotation2d(transform.getX(), transform.getY());
    }

    public static Rotation2d getDirection(Pose2d tail, Pose2d head) {
        return getDirection(head.getTranslation().minus(tail.getTranslation()));
    }

    public static double getDistance(Transform2d transform){
        return getDistance(transform.getTranslation());
    }

    public static double getDistance(Translation2d transform) {
        return transform.getNorm();
    }

    public static double calculateDistanceToTargetMeters(
        double cameraHeightMeters,
        double targetHeightMeters,
        double cameraPitchRadians,
        double targetPitchRadians,
        double cameraYawRadians) {
    return (targetHeightMeters - cameraHeightMeters)
            / Math.tan(cameraPitchRadians + targetPitchRadians) / Math.cos(cameraYawRadians);
    }

    /**
     * Gets the rotation of a Rotation2d in the range [0..2pi] instead of the default [-pi..pi]. 
     * 
     * This avoids wrapping problems 
     * @param rotation
     * @return
     */
    public static double modulus(Rotation2d rotation) {
        return modulus(rotation.getRadians());
    }

    public static double modulus(double rotation) {
        if(rotation < 0) {
        rotation = 2*Math.PI + rotation;
        }
        return rotation;
    }

    public static double subtractkS(double voltage, double kS) {
        if(Math.abs(voltage) <= kS) {
            voltage = 0;
        }
        else {
            voltage -= Math.copySign(kS, voltage);
        }
        return voltage;
    }

    
}
