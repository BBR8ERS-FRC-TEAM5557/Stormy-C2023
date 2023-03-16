package org.team5557.subsystems.elevator.commands;

import org.library.team254.motion.MotionProfileGoal;
import org.team5557.RobotContainer;
import org.team5557.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorHeight extends CommandBase {
    Elevator elevator = RobotContainer.elevator;
    double setpoint;
    MotionProfileGoal goal;

    public SetElevatorHeight(double setpoint) {
        this.setpoint = setpoint;
        this.goal = new MotionProfileGoal(setpoint);
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setMotionProfilingGoal(goal);
    }
    
}
