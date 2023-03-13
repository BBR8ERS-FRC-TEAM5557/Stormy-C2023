package org.team5557.subsystems.wrist.commands;

import java.util.function.DoubleSupplier;

import org.team5557.RobotContainer;
import org.team5557.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristManual extends CommandBase {
    private final Wrist shoulder = RobotContainer.wrist;
    private final DoubleSupplier wristJogger;

    private final double maxPower = 5.0;

    public WristManual(DoubleSupplier wristJogger) {
        ShuffleboardTab tab = Shuffleboard.getTab("Wrist");
        this.wristJogger = wristJogger;
        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        shoulder.setOpenLoop(wristJogger.getAsDouble() / maxPower);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.setOpenLoop(0.0);
    }
}