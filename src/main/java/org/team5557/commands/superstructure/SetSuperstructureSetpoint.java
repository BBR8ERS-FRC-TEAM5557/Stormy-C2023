package org.team5557.commands.superstructure;

import java.util.function.Supplier;

import org.library.team254.motion.MotionProfileConstraints;
import org.library.team254.motion.MotionState;
import org.library.team254.motion.SetpointGenerator;
import org.library.team254.motion.SetpointGenerator.Setpoint;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.planners.TuckPlanner;
import org.team5557.planners.arm.ArmDynamics;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.subsystems.elevator.Elevator;
import org.team5557.subsystems.shoulder.Shoulder;
import org.team5557.subsystems.wrist.Wrist;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSuperstructureSetpoint extends CommandBase {
    Supplier<SuperstructureState> stateSupplier;

    private final ArmDynamics dynamics = RobotContainer.arm_dynamics;
    private final Elevator elevator = RobotContainer.elevator;
    private final Shoulder shoulder = RobotContainer.shoulder;
    private final Wrist wrist = RobotContainer.wrist;

    private double timestamp = 0.0;

    private double shoulderDemand;
    protected SetpointGenerator mShoulderSetpointGenerator = new SetpointGenerator();
    protected MotionProfileConstraints mShoulderMotionProfileConstraints;
    protected MotionState mShoulderMotionStateSetpoint = null;
    protected TrapezoidProfile.State mShoulderPreviousProfiledState = null;

    public SetSuperstructureSetpoint(SuperstructureState state) {
        this(() -> state);
    }
    public SetSuperstructureSetpoint(Supplier<SuperstructureState> stateSupplier) {
        this.stateSupplier = stateSupplier;
    }

    @Override
    public void initialize() {
        timestamp = Timer.getFPGATimestamp();
        
        //TuckPlanner.plan(null, stateSupplier.get());
        SuperstructureState desState = stateSupplier.get();
        SuperstructureState prevState = new SuperstructureState(
            elevator.getPosition(),
            shoulder.getPosition(),
            wrist.getPosition()
        );
        
        desState = TuckPlanner.plan(prevState, desState);

        mShoulderMotionStateSetpoint = new MotionState(timestamp, shoulder.getPosition(), shoulder.getVelocity(), 0.0);
        mShoulderSetpointGenerator.reset();

        Setpoint shoulderSetpoint = mShoulderSetpointGenerator.getSetpoint(null, null, null, timestamp + Constants.kloop_period);
        shoulderDemand = 0.0;//shoulder.constrainUnits(shoulderSetpoint.motion_state.pos());
        mShoulderMotionStateSetpoint = shoulderSetpoint.motion_state;

        Vector<N2> feedforward = dynamics.feedforward(
            VecBuilder.fill(shoulderDemand, 0), 
            VecBuilder.fill(shoulderSetpoint.motion_state.vel(), 0), 
            VecBuilder.fill(0.0, 0.0)
        );

        shoulder.setSetpointPositionPID(0, feedforward.get(0, 0));
        wrist.setSetpointPositionPID(0, feedforward.get(1, 0));

    }
    
}
