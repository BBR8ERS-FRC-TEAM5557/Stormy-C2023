package org.library.team2910.control;

/**
 * A constraint that limits the velocity.
 */
public class MaxVelocityConstraint extends TrajectoryConstraint {
    private final double maxVelocity;

    public MaxVelocityConstraint(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    @Override
    public double getMaxVelocity(Path.State state) {
        return maxVelocity;
    }
}
