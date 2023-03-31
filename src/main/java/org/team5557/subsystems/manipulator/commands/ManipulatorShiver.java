package org.team5557.subsystems.manipulator.commands;

import org.library.team254.util.LatchedBoolean;
import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.Intake;
import org.team5557.subsystems.manipulator.Manipulator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.team5557.subsystems.intake.util.IntakeSubsystemConstants.*;

public class ManipulatorShiver extends CommandBase {
    Manipulator manipulator = RobotContainer.manipulator;
    XboxController controller = RobotContainer.primary_controller;

    Timer t = new Timer();
    LatchedBoolean gate = new LatchedBoolean();

    @Override
    public void initialize() {
        t.stop();
        t.reset();
    }

    @Override
    public void execute() {
        if(gate.update(manipulator.getCubeDetected())) {
            t.start();
            controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverHard);
        }

        if(t.get() > kShiverTime) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            t.stop();
            t.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
    
}
