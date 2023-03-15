package org.team5557.auto;

import org.library.team2713.AutoPath;
import org.team5557.Constants;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {

    //private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final PathPlannerTrajectory RpushAndCharge;
    private final PathPlannerTrajectory BpushAndCharge;

    private final AutoPath NoBump_3_Charge;

    public AutonomousTrajectories() {
        BpushAndCharge = PathPlanner.loadPath("BpushAndCharge", Constants.pathplanner.medium_constraints);
        RpushAndCharge = PathPlanner.loadPath("RpushAndCharge", Constants.pathplanner.medium_constraints);

        NoBump_3_Charge = new AutoPath("NoBump_3_Charge", Constants.pathplanner.medium_constraints);
    }

    public PathPlannerTrajectory getRPushAndCharge() {
        return RpushAndCharge;
    }

    public PathPlannerTrajectory getBPushAndCharge() {
        return BpushAndCharge;
    }

    public PathPlannerTrajectory getNoBump_3_Charge() {
        return NoBump_3_Charge.getTrajectory();
    }
}