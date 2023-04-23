package org.team5557.auto;

import org.library.team2713.AutoPath;
import org.team5557.Constants;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {

    //private static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);
    private final AutoPath spit_Charge_Center;
    private final AutoPath twoplusoneChargeNeie;
    private final AutoPath twoParkNee;
    private final AutoPath threeParkNeii;
    private final AutoPath twoChargeAddNei;
    private final AutoPath twoChargeAddNee;
    private final AutoPath twoParkNei;
    private final AutoPath twoChargeAddBei;
    private final AutoPath twoParkBei;
    private final AutoPath oneplusoneChargeCeiPart1;
    private final AutoPath score_Charge_Center;
    private final AutoPath oneChargeCePart2;
    private final AutoPath oneplusoneChargeCeiPart3;


    public AutonomousTrajectories() {
        threeParkNeii = new AutoPath("3-PARK-N-(eii)", Constants.pathplanner.auto_constraints);
        twoplusoneChargeNeie = new AutoPath("2+1-Charge-N-(eie)", Constants.pathplanner.auto_constraints);

        twoParkNee = new AutoPath("2-Park-N-(ee)", Constants.pathplanner.auto_constraints);
        twoChargeAddNee = new AutoPath("2-ChargeAdd-N-(ee)", Constants.pathplanner.fast_constraints);

        twoParkNei = new AutoPath("2-Park-N-(ei)", Constants.pathplanner.auto_constraints);
        twoChargeAddNei = new AutoPath("2-ChargeAdd-N-(ei)", Constants.pathplanner.fast_constraints);

        twoParkBei = new AutoPath("2-Park-B-(ei)", Constants.pathplanner.auto_constraints);
        twoChargeAddBei = new AutoPath("2-ChargeAdd-B-(ei)", Constants.pathplanner.fast_constraints);

        oneplusoneChargeCeiPart1 = new AutoPath("1+1-Charge-C-(ei)-part1", Constants.pathplanner.fast_constraints);
        oneplusoneChargeCeiPart3 = new AutoPath("1+1-Charge-C-(ei)-part3", Constants.pathplanner.fast_constraints);


        oneChargeCePart2 = new AutoPath("1-Charge-C-(e)-part2", Constants.pathplanner.fast_constraints);
        score_Charge_Center = new AutoPath("1-Charge-C(e)", Constants.pathplanner.fast_constraints);
        spit_Charge_Center = new AutoPath("BpushAndCharge", Constants.pathplanner.fast_constraints);
    }

    public PathPlannerTrajectory get3ParkNeii() {
        return threeParkNeii.getTrajectory();
    }
    public PathPlannerTrajectory get21ChargeNeie() {
        return twoplusoneChargeNeie.getTrajectory();
    }


    public PathPlannerTrajectory get2ParkNee() {
        return twoParkNee.getTrajectory();
    }
    public PathPlannerTrajectory get2ChargeAddNee() {
        return twoChargeAddNee.getTrajectory();
    }


    public PathPlannerTrajectory get2ParkNei() {
        return twoParkNei.getTrajectory();
    }   
    public PathPlannerTrajectory get2ChargeAddNei() {
        return twoChargeAddNei.getTrajectory();
    }


    public PathPlannerTrajectory get2ParkBei() {
        return twoParkBei.getTrajectory();
    }   
    public PathPlannerTrajectory get2ChargeAddBei() {
        return twoChargeAddBei.getTrajectory();
    }


    public PathPlannerTrajectory get11ChargeCeiPart1() {
        return oneplusoneChargeCeiPart1.getTrajectory();
    }   
    public PathPlannerTrajectory get11ChargeCeiPart3() {
        return oneplusoneChargeCeiPart3.getTrajectory();
    }


    public PathPlannerTrajectory getScore_Charge_Center() {
        return score_Charge_Center.getTrajectory();
    }
    public PathPlannerTrajectory getOneChargeCePart2() {
        return oneChargeCePart2.getTrajectory();
    }
    public PathPlannerTrajectory getSpit_Charge_Center() {
        return spit_Charge_Center.getTrajectory();
    }

}