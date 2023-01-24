package org.team5557.auto;

import org.team5557.Constants;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {

    //private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final PathPlannerTrajectory testAutoPartOne;

    public AutonomousTrajectories() {
        testAutoPartOne = PathPlanner.loadPath(null, Constants.pathplanner.hellaslow_constraints);
    }

    public PathPlannerTrajectory getTestAutoPartOne() {
        return testAutoPartOne;
    }
}