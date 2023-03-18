package org.team5557.subsystems.manipulator.commands;

import org.team5557.RobotContainer;
import org.team5557.state.goal.ObjectiveTracker;
import org.team5557.state.goal.ObjectiveTracker.NodeLevel;
import org.team5557.subsystems.manipulator.Manipulator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartEject extends CommandBase {
    ObjectiveTracker ot = RobotContainer.objective_tracker;

    @Override
    public void initialize() {
        if(ot.selectedColumn == 1 || ot.selectedColumn == 4 || ot.selectedColumn == 7 || ot.selectedLevel.equals(NodeLevel.HYBRID)) {
            ManipulatorAuto.ejectCube().schedule();
        } else {
            ManipulatorAuto.ejectCubeHigh().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
