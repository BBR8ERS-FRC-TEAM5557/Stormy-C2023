package org.team5557.planners.arm;

import edu.wpi.first.math.system.plant.DCMotor;

public class JointConfig {
    public final double mass;
    public final double length;
    public final double moi;
    public final double cgRadius;
    public final double minAngle;
    public final double maxAngle;
    public final double reduction;
    public final DCMotor motor;

    public JointConfig(
            double mass,
            double length,
            double moi,
            double cgRadius,
            double minAngle,
            double maxAngle,
            double reduction,
            DCMotor motor) {
        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.cgRadius = cgRadius;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.reduction = reduction;
        this.motor = motor;
    }
}
