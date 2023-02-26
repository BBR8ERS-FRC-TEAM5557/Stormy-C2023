package org.team5557.planners;

import org.library.team2910.math.MathUtils;
import org.library.team2910.util.InterpolatingDouble;
import org.library.team2910.util.InterpolatingTreeMap;
import org.team5557.Constants;
import org.team5557.planners.superstructure.util.SuperstructureConstants;
import org.team5557.planners.superstructure.util.SuperstructureState;

public class TuckPlanner {

    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kTuckedArmToWristAngle = new InterpolatingTreeMap<>();
    static {
        kTuckedArmToWristAngle.put(new InterpolatingDouble(160.0), new InterpolatingDouble(30.0));
        kTuckedArmToWristAngle.put(new InterpolatingDouble(90.0), new InterpolatingDouble(180.0));
        kTuckedArmToWristAngle.put(new InterpolatingDouble(0.0), new InterpolatingDouble(330.0));
    }

    public static SuperstructureState plan(SuperstructureState prev, SuperstructureState goal) {
        /*
        double carriage_interference_point_shoulder_angle = 90.0;
        double wrist_angle_to_be_tucked = 140.0;

        boolean shoulder_past_interference_point = prev.shoulder > carriage_interference_point_shoulder_angle + SuperstructureConstants.kShoulderPaddingDegrees;
        boolean needs_tuck = goal.shoulder > carriage_interference_point_shoulder_angle + SuperstructureConstants.kShoulderPaddingDegrees && !shoulder_past_interference_point;
        boolean prev_setpoint_wrist_tucked = MathUtils.epsilonEquals(prev.wrist, wrist_angle_to_be_tucked,
                SuperstructureConstants.kWristPaddingDegrees);


        if(needs_tuck && !prev_setpoint_wrist_tucked) {
            SuperstructureState result = new SuperstructureState(makeLegal(goal));
            result.wrist = wrist_angle_to_be_tucked;
            result.shoulder = prev.shoulder;
            return result;
        }

        boolean shoulder_past_interference_point2 = prev.shoulder < carriage_interference_point_shoulder_angle - SuperstructureConstants.kShoulderPaddingDegrees;
        boolean needs_tuck2 = goal.shoulder < carriage_interference_point_shoulder_angle - SuperstructureConstants.kShoulderPaddingDegrees && !shoulder_past_interference_point2;
        if(needs_tuck2 && !prev_setpoint_wrist_tucked) {
            SuperstructureState result = new SuperstructureState(makeLegal(goal));
            result.wrist = wrist_angle_to_be_tucked;
            result.shoulder = prev.shoulder;
            return result;
        }

        return goal;*/
        /*
        boolean crosses_transition = (prev.shoulder <= intersection_point_min && goal.shoulder >= intersection_point_min) || (prev.shoulder >= intersection_point_max && goal.shoulder <= intersection_point_max);

        if(crosses_transition) {
            SuperstructureState result = new SuperstructureState(makeLegal(goal));
            result.wrist = kTuckedArmToWristAngle.getInterpolated(new InterpolatingDouble(prev.shoulder)).value;
            if(MathUtils.epsilonEquals(prev.wrist, result.wrist, SuperstructureConstants.kWristPaddingDegrees)) {
                result.shoulder = goal.shoulder;
                return result;
            } else {
                result.shoulder = prev.shoulder;
            }
        }*/
        return goal;
    }

    private static SuperstructureState makeLegal(SuperstructureState state) {
        SuperstructureState result = new SuperstructureState(state);
        result.shoulder = MathUtils.clamp(result.shoulder, Constants.kShoulderConstants.minAngle, Constants.kShoulderConstants.maxAngle);
        //result.elevator = MathUtils.clamp(result.elevator, Constants.kElevatorConstants.kMinUnitsLimit, Constants.kElevatorConstants.kMaxUnitsLimit);
        result.wrist = MathUtils.clamp(result.wrist, Constants.kWristConstants.minAngle, Constants.kWristConstants.maxAngle);
        return result;
    }
    
}
