package org.team5557.subsystems.wrist.util;

public class WristConversions {

    public static double encoderRelToShoulderRel(double encoderRel, double shoulderPos) {
        return (2.0 * (270.0 - shoulderPos)) + encoderRel - 180.0;
    }

    public static double shoulderRelToEncoder(double shoulderRel, double shoulderPos) {
        return 180.0 + shoulderRel - (2.0 * (270.0 - shoulderPos));
    }
    
}
