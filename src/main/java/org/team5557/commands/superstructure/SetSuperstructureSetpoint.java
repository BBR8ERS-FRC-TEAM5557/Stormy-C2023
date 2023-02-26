package org.team5557.commands.superstructure;

import java.util.function.Supplier;

import org.team5557.planners.TuckPlanner;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.state.goal.ObjectiveTracker.NodeLevel;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSuperstructureSetpoint extends CommandBase {
    Supplier<SuperstructureState> stateSupplier;

    public SetSuperstructureSetpoint(SuperstructureState state) {
        this(() -> state);
    }
    public SetSuperstructureSetpoint(Supplier<SuperstructureState> stateSupplier) {
        this.stateSupplier = stateSupplier;
    }

    @Override
    public void initialize() {
        //TuckPlanner.plan(null, stateSupplier.get());
    }
    
}
