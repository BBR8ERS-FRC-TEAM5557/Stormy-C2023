package org.team5557.subsystems.manipulator.commands;

import org.team5557.RobotContainer;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.state.goal.ObjectiveTracker;
import org.team5557.state.goal.ObjectiveTracker.NodeLevel;
import org.team5557.subsystems.manipulator.Manipulator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartEject extends CommandBase {
    RobotStateSupervisor state = RobotContainer.state_supervisor;

    @Override
    public void initialize() {
        if(state.getColumn() == 1 || state.getColumn() == 4 || state.getColumn() == 7) {
            ManipulatorAuto.ejectCube().schedule();
        } else {
            ManipulatorAuto.ejectCone().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
