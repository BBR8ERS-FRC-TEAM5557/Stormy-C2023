package org.team5557.subsystems.shoulder.commands;

import java.util.function.DoubleSupplier;

import org.team5557.RobotContainer;
import org.team5557.subsystems.shoulder.Shoulder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShoulderManual extends CommandBase {
    private final Shoulder shoulder = RobotContainer.shoulder;
    private final DoubleSupplier shoulderJogger;

    private final double maxPower = 2.0;

    public ShoulderManual(DoubleSupplier shoulderJogger) {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        this.shoulderJogger = shoulderJogger;
        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        shoulder.setOpenLoop(shoulderJogger.getAsDouble() / maxPower);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.setOpenLoop(0.0);
    }
}
