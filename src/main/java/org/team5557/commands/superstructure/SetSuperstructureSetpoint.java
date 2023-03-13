package org.team5557.commands.superstructure;

import java.util.function.Supplier;

import org.library.team254.motion.MotionProfileGoal;
import org.library.team254.motion.MotionState;
import org.library.team254.motion.SetpointGenerator;
import org.library.team254.motion.SetpointGenerator.Setpoint;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.planners.TuckPlanner;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.subsystems.elevator.Elevator;
import org.team5557.subsystems.shoulder.Shoulder;
import org.team5557.subsystems.shoulder.util.ShoulderSubsystemConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSuperstructureSetpoint extends CommandBase {
    Supplier<SuperstructureState> stateSupplier;

    private final Elevator elevator = RobotContainer.elevator;
    private final Shoulder shoulder = RobotContainer.shoulder;

    private double timestamp = 0.0;

    private final SetpointGenerator mShoulderSetpointGenerator = new SetpointGenerator();
    private MotionState mShoulderMotionStateSetpoint = null;


    public SetSuperstructureSetpoint(SuperstructureState state) {
        this(() -> state);
    }
    public SetSuperstructureSetpoint(Supplier<SuperstructureState> stateSupplier) {
        this.stateSupplier = stateSupplier;
    }

    @Override
    public void initialize() {
        timestamp = Timer.getFPGATimestamp();

        mShoulderSetpointGenerator.reset();
        mShoulderMotionStateSetpoint = new MotionState(timestamp, shoulder.getPosition(), 0.0, 0.0);//shoulder.getVelocity(), 0.0);
    }

    @Override
    public void execute() {
        timestamp = Timer.getFPGATimestamp();
        
        //TuckPlanner.plan(null, stateSupplier.get());
        SuperstructureState desState = stateSupplier.get();
        SuperstructureState prevState = new SuperstructureState(
            elevator.getPosition(),
            shoulder.getAngle()
        );
        
        desState = TuckPlanner.plan(prevState, desState);

        //Shoulder
        MotionProfileGoal shoulderGoal = new MotionProfileGoal(desState.shoulder);
        Setpoint shoulderSetpoint = mShoulderSetpointGenerator.getSetpoint(ShoulderSubsystemConstants.motionConstraints, shoulderGoal, mShoulderMotionStateSetpoint, timestamp + Constants.kloop_period);
        double shoulderDemand = shoulder.constrainUnits(shoulderSetpoint.motion_state.pos());
        mShoulderMotionStateSetpoint = shoulderSetpoint.motion_state;


        elevator.setMotionProfilingGoal(0.0);
        shoulder.setSetpointPositionPID(shoulderDemand, 0.0);

    }
}
