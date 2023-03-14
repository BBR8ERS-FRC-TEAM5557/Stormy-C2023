package org.team5557.subsystems.intake.commands;

import org.library.team254.util.LatchedBoolean;
import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.team5557.subsystems.intake.util.IntakeSubsystemConstants.*;

public class IntakeShiver extends CommandBase {
    Intake intake = RobotContainer.intake;
    XboxController controller = RobotContainer.danny_controller;

    Timer t = new Timer();
    LatchedBoolean gate = new LatchedBoolean();

    @Override
    public void initialize() {
        t.stop();
        t.reset();
        controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverSoft);
    }

    @Override
    public void execute() {
        if(gate.update(intake.getCubeDetected())) {
            t.start();
            controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverHard);
        }

        if(t.get() > kShiverTime) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverSoft);
            t.stop();
            t.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
    
}
