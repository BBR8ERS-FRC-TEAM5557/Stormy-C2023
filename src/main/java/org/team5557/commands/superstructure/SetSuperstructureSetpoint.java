package org.team5557.commands.superstructure;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.library.team254.motion.MotionProfileGoal;
import org.team5557.RobotContainer;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.subsystems.elevator.Elevator;
import org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants;
import org.team5557.subsystems.shoulder.Shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSuperstructureSetpoint extends CommandBase {
    Supplier<SuperstructureState> stateSupplier;

    private final Elevator elevator = RobotContainer.elevator;
    private final Shoulder shoulder = RobotContainer.shoulder;

    private final DoubleSupplier elevatorJogger;

    private SuperstructureState state, currentState;
    private MotionProfileGoal shoulderState;
    private MotionProfileGoal elevatorState;

    private final double kMaxClearHeight = 12.0;
    private final double kArmSafeAngle = 240.0;


    public SetSuperstructureSetpoint(SuperstructureState state) {
        this(() -> state, ()-> 0.0);
    }

    public SetSuperstructureSetpoint(SuperstructureState state, DoubleSupplier elevatorJogger) {
        this(() -> state, elevatorJogger);
    }

    public SetSuperstructureSetpoint(Supplier<SuperstructureState> stateSupplier, DoubleSupplier elevatorJogger) {
        this.stateSupplier = stateSupplier;
        this.elevatorJogger = elevatorJogger;
        addRequirements(shoulder);
    }

    public SetSuperstructureSetpoint(Supplier<SuperstructureState> stateSupplier) {
        this.stateSupplier = stateSupplier;
        this.elevatorJogger = () -> 0.0;
        addRequirements(shoulder, elevator);
    }

    @Override
    public void execute() {
        state = stateSupplier.get();
        currentState = RobotContainer.state_supervisor.getCurrentSuperstructureState();

        double elevatorDelta = currentState.elevator - state.elevator;
        double armDelta = currentState.shoulder - state.shoulder;

        /*
        if(Math.abs(elevatorJogger.getAsDouble()) < 0.1) {
            elevator.setMotionProfilingGoal(elevatorState);
        } else {
            elevator.setOpenLoop(elevatorJogger.getAsDouble() * ElevatorSubsystemConstants.kMaxManualPower);
        }*/
        //elevator.setOpenLoop(elevatorJogger.getAsDouble() * ElevatorSubsystemConstants.kMaxManualPower);
        if(state.elevator > kMaxClearHeight && Math.abs(elevatorDelta) > 15.0) {
            shoulderState = new MotionProfileGoal(kArmSafeAngle);
        }
        else {
            shoulderState = new MotionProfileGoal(state.shoulder);
        }

        //shoulderState = new MotionProfileGoal(state.shoulder);
        shoulder.setMotionProfilingGoal(shoulderState);
        elevatorState = new MotionProfileGoal(state.elevator);
        elevator.setMotionProfilingGoal(elevatorState);
    }

    public boolean isAtDesiredState() {
        return state.isAtDesiredState(currentState); 
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        shoulder.stop();
    }

}
