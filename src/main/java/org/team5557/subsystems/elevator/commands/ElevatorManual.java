package org.team5557.subsystems.elevator.commands;

import java.util.function.DoubleSupplier;

import org.team5557.RobotContainer;
import org.team5557.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorManual extends CommandBase {
    private final Elevator elevator = RobotContainer.elevator;
    private final DoubleSupplier elevatorJogger;

    private final double maxPower = 10.0;

    public ElevatorManual(DoubleSupplier elevatorJogger) {
        this.elevatorJogger = elevatorJogger;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setOpenLoop(elevatorJogger.getAsDouble() / maxPower);
    }
}
