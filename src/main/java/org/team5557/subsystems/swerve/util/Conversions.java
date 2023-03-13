package org.team5557.subsystems.swerve.util;

public class Conversions {
    /**
     * @param counts    NEO Rotations
     * @param gearRatio Gear Ratio between NEO and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double rotationsToDegrees(double rotations, double gearRatio) {
        return rotations * (360.0 / gearRatio);
    }

    /**
     * This is a commonly used function when setting the range of a radian measurement from [-pi, pi)
     * to [0,2pi]. This is requred a lot when ensuring that the number being sent in to the NEO's onboard
     * angle PID Controller is within the range necessary. Note that this also performs modular dision first
     * on the angle which allows for an angle like -3pi be sent in, and return 1pi.
     * @param radians - the 
     * @return the same angle in the range [0,2pi)
     */
    public static double convertPiPositive(double radians) {
        double angle = radians;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    /**
    * @param velocity Velocity MPS
    * @param circumference Circumference of Wheel
    * @param gearRatio Gear Ratio between Motor and Mechanism (set to 1 for Falcon RPM)
    * @return Falcon Velocity Counts
    */
    public static double mpsToRPM(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60.0) / circumference);
        return wheelRPM * gearRatio;
    }

    /**
     * @param velocityrotations Motor Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Wheel Meters per second
     */
    public static double rpmToMPS(double velocityrotations, double circumference, double gearRatio) {
    double wheelRPM = velocityrotations/gearRatio;
    return (wheelRPM * circumference) / 60.0;
    }

    /**
     * @param counts NEO rotations
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Motor and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
        double wheelRotations = rotations / gearRatio;
        return (wheelRotations * circumference);
    }

}
