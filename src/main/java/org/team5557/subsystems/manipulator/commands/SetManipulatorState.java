package org.team5557.subsystems.manipulator.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.manipulator.Manipulator;
import org.team5557.subsystems.manipulator.util.ManipulatorState;
import org.team5557.subsystems.manipulator.util.ManipulatorState.ManipulatorStates;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetManipulatorState extends CommandBase {

    private final Manipulator manipulator = RobotContainer.manipulator;
    private final ManipulatorState state;
    
    public SetManipulatorState(ManipulatorState state) {
        this.state = state;
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.setManipulatorState(state);
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.setManipulatorState(new ManipulatorState(0, manipulator.getManipulatorState().observedPiece));
    }
}
