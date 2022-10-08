package frc.robot.util.interpolation;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double backWheelRpm;
    public final double frontWheelRpm;

    // Constructor
    public ShotParameter(double frontWheelRpm, double backWheelRpm) {
        this.frontWheelRpm = frontWheelRpm;
        this.backWheelRpm = backWheelRpm;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.frontWheelRpm - other.frontWheelRpm) < 0.1 &&
        Math.abs(this.backWheelRpm - other.backWheelRpm) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(frontWheelRpm, end.frontWheelRpm, t), 
            lerp(backWheelRpm, end.backWheelRpm, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}