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
        SuperstructureState state = stateSupplier.get();

        /*MotionProfileGoal elevatorState = new MotionProfileGoal(state.elevator);

        if(Math.abs(elevatorJogger.getAsDouble()) < 0.1) {
            elevator.setMotionProfilingGoal(elevatorState);
        } else {
            elevator.setOpenLoop(elevatorJogger.getAsDouble() * ElevatorSubsystemConstants.kMaxManualPower);
        }*/
        //elevator.setOpenLoop(elevatorJogger.getAsDouble() * ElevatorSubsystemConstants.kMaxManualPower);

        MotionProfileGoal shoulderState = new MotionProfileGoal(state.shoulder);
        //shoulder.setMotionProfilingGoal(shoulderState);
    }
}
