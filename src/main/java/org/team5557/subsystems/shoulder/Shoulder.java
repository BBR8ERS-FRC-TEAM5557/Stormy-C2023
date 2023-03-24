package org.team5557.subsystems.shoulder;

import org.library.team254.drivers.ServoMotorSubsystemAbs;
import org.library.team254.motion.IMotionProfileGoal;
import org.library.team254.motion.MotionState;
import org.library.team254.motion.SetpointGenerator.Setpoint;
import org.library.team2910.math.MathUtils;
import static org.team5557.subsystems.shoulder.util.ShoulderSubsystemConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shoulder extends ServoMotorSubsystemAbs {
    ArmFeedforward feedforward;

    public Shoulder(ServoMotorSubsystemAbsConstants constants) {
        super(constants);

        feedforward = new ArmFeedforward(kS, kG, kV, kA);

        ShuffleboardTab tab = Shuffleboard.getTab("Shoulder");
        tab.add(this);
        tab.addNumber("Angle", () -> getAngle());
        tab.addNumber("Setpoint", () -> getSetpoint());
        tab.addNumber("Velocity", () -> getVelocity());

        mMasterEncoder.setPositionConversionFactor(360.0);
        mMasterEncoder.setVelocityConversionFactor(360.0 / 60.0);
        mMasterEncoder.setZeroOffset(139.9736);

        mMotionProfileConstraints = motionConstraints;
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return MathUtils.epsilonEquals(mPeriodicIO.position_units, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }

    public double constrainUnits(double units) {
        return super.constrainUnits(units);
    }

    public synchronized void setMotionProfilingGoal(IMotionProfileGoal goal) {
        if (mControlState != ControlState.MOTION_PROFILING_254) {
            mControlState = ControlState.MOTION_PROFILING_254;
            mMotionStateSetpoint = new MotionState(mPeriodicIO.timestamp, mPeriodicIO.position_units, mPeriodicIO.velocity_units_per_s, 0.0);
            mSetpointGenerator.reset();
        }
        Setpoint setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraints, goal, mMotionStateSetpoint, mPeriodicIO.timestamp + mConstants.kLooperDt);
        mPeriodicIO.demand = constrainUnits(setpoint.motion_state.pos());
        mPeriodicIO.feedforward = MathUtils.clamp(feedforward.calculate(
            Units.degreesToRadians(mPeriodicIO.demand), 
            Units.degreesToRadians(setpoint.motion_state.vel()), 
            Units.degreesToRadians(setpoint.motion_state.acc())), -2, 2);
        mMotionStateSetpoint = setpoint.motion_state;
    }
    
}
