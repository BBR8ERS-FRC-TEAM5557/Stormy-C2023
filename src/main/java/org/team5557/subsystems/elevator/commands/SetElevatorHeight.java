package org.team5557.subsystems.elevator.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorHeight extends CommandBase {
    Elevator elevator = RobotContainer.elevator;
    double setpoint;

    public SetElevatorHeight(double setpoint) {
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setSetpointPositionPID(12.0);
    }
    
}
