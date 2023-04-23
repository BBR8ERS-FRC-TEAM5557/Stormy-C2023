package org.team5557.subsystems.manipulator.commands;

import org.library.team254.util.LatchedBoolean;
import org.team5557.RobotContainer;
import org.team5557.state.goal.ObjectiveTracker.GamePiece;
import org.team5557.state.vision.util.DetectedObject;
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
    boolean isCone;

    public ManipulatorShiver(boolean isCone) {
        this.isCone = isCone;
    }

    @Override
    public void initialize() {
        t.stop();
        t.reset();
    }

    @Override
    public void execute() {
        DetectedObject object = RobotContainer.state_supervisor.getDetectedObject();
        if(object.gamePiece == GamePiece.CONE && isCone) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if(object.gamePiece == GamePiece.CUBE && !isCone) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }
        if (gate.update(manipulator.getCubeDetected() || manipulator.getConeDetected())) {
            t.start();
            controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverHard);
        }
        if (t.get() > 0.1) {
            if (t.get() < 0.2) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverHard);
            } else if (t.get() < 0.4) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            } else if (t.get() < 0.6) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverHard);
            } else if (t.get() < 0.8) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            } else if (t.get() < 1.0) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, kShiverHard);
            } else {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                t.stop();
                t.reset();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

}
