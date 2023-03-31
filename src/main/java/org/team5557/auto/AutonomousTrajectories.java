package org.team5557.auto;

import org.library.team2713.AutoPath;
import org.team5557.Constants;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {

    //private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);
    private final AutoPath spit_Charge_Center;

    private final AutoPath three_Charge_NoBump;
    private final AutoPath three_Park_NoBump;
    private final AutoPath twoplusone_Charge_NoBump;
    private final AutoPath oneplusone_Charge_Bump;

    private final AutoPath i_Three_Charge_NoBump;
    private final AutoPath i_Three_Park_NoBump;
    private final AutoPath i_Twoplusone_Charge_NoBump;

    private final PathPlannerTrajectory RpushAndCharge;
    private final PathPlannerTrajectory BpushAndCharge;
    private final AutoPath one_Park_Bump;
    private final AutoPath oneplusone_Park_NoBump;

    public AutonomousTrajectories() {
        spit_Charge_Center = new AutoPath("BpushAndCharge", Constants.pathplanner.medium_constraints);

        three_Charge_NoBump = new AutoPath("3_Charge_NoBump", Constants.pathplanner.medium_constraints);
        three_Park_NoBump = new AutoPath("3_Park_NoBump", Constants.pathplanner.medium_constraints);
        twoplusone_Charge_NoBump = new AutoPath("2+1_Charge_NoBump", Constants.pathplanner.medium_constraints);
        oneplusone_Charge_Bump = new AutoPath("1+1_Charge_Bump", Constants.pathplanner.medium_constraints);

        i_Three_Charge_NoBump = new AutoPath("I_3_Charge_NoBump", Constants.pathplanner.medium_constraints);
        i_Three_Park_NoBump = new AutoPath("I_3_Park_NoBump", Constants.pathplanner.medium_constraints);
        i_Twoplusone_Charge_NoBump = new AutoPath("I_2+1_Charge_NoBump", Constants.pathplanner.medium_constraints);

        BpushAndCharge = PathPlanner.loadPath("BpushAndCharge", Constants.pathplanner.medium_constraints);
        RpushAndCharge = PathPlanner.loadPath("RpushAndCharge", Constants.pathplanner.medium_constraints);

        one_Park_Bump = new AutoPath("1_Park_Bump", Constants.pathplanner.medium_constraints);
        oneplusone_Park_NoBump = new AutoPath("1+1_Park_NoBump", Constants.pathplanner.medium_constraints);
    }


    public PathPlannerTrajectory getSpit_Charge_Center() {
        return spit_Charge_Center.getTrajectory();
    }

    //REGULAR
    public PathPlannerTrajectory get3_Charge_NoBump() {
        return three_Charge_NoBump.getTrajectory();
    }
    public PathPlannerTrajectory get3_Park_NoBump() {
        return three_Park_NoBump.getTrajectory();
    }
    public PathPlannerTrajectory get2plus1_Charge_NoBump() {
        return twoplusone_Charge_NoBump.getTrajectory();
    }
    public PathPlannerTrajectory get1plus1_Charge_Bump() {
        return oneplusone_Charge_Bump.getTrajectory();
    }

    ///INTAK ONLY STUFF
    public PathPlannerTrajectory getI_3_Charge_NoBump() {
        return i_Three_Charge_NoBump.getTrajectory();
    }
    public PathPlannerTrajectory getI_3_Park_NoBump() {
        return i_Three_Park_NoBump.getTrajectory();
    }
    public PathPlannerTrajectory getI_2plus1_Charge_NoBump() {
        return i_Twoplusone_Charge_NoBump.getTrajectory();
    }

    ///RANDOM SHit

    public PathPlannerTrajectory get1_Park_Bump() {
        return one_Park_Bump.getTrajectory();
    }

    public PathPlannerTrajectory get1plus1_Park_NoBump() {
        return oneplusone_Park_NoBump.getTrajectory();
    }
    public PathPlannerTrajectory getRPushAndCharge() {
        return RpushAndCharge;
    }

    public PathPlannerTrajectory getBPushAndCharge() {
        return BpushAndCharge;
    }
}