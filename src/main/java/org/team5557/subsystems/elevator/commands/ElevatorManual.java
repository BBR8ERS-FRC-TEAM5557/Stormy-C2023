package org.team5557.subsystems.elevator.commands;

import java.util.function.DoubleSupplier;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.elevator.Elevator;
import org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorManual extends CommandBase {
    private final Elevator elevator = RobotContainer.elevator;
    private final DoubleSupplier elevatorJogger;

    public ElevatorManual(DoubleSupplier elevatorJogger) {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.elevator_readout_key);
        this.elevatorJogger = elevatorJogger;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setOpenLoop(elevatorJogger.getAsDouble() * ElevatorSubsystemConstants.kMaxManualPower);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setOpenLoop(0.0);
    }
}
