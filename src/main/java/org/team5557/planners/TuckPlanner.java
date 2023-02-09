package org.team5557.planners;

import org.library.team2910.math.MathUtils;
import org.team5557.Constants;
import org.team5557.planners.superstructure.util.SuperstructureConstants;
import org.team5557.planners.superstructure.util.SuperstructureState;

public class TuckPlanner {


    public static SuperstructureState plan(SuperstructureState prev, SuperstructureState goal) {
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
