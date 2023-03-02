package org.team5557.auto;

import java.util.List;

import org.team5557.Constants;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Filesystem;

public class AutonomousTrajectories {

    //private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final PathPlannerTrajectory RpushAndCharge;
    private final PathPlannerTrajectory BpushAndCharge;

    public AutonomousTrajectories() {
        BpushAndCharge = PathPlanner.loadPath("BpushAndCharge.path", Constants.pathplanner.medium_constraints);
        RpushAndCharge = PathPlanner.loadPath("RpushAndCharge.path", Constants.pathplanner.medium_constraints);
    }

    public PathPlannerTrajectory getRPushAndCharge() {
        return RpushAndCharge;
    }

    public PathPlannerTrajectory getBPushAndCharge() {
        return BpushAndCharge;
    }
}