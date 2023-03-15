package org.team5557.subsystems.intake.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PassThrough extends SequentialCommandGroup {
    Intake intake = RobotContainer.intake;
    
    public PassThrough() {
        addCommands(
            IntakeAuto.startIntaking(),
            new WaitCommand(100).until(intake::getCubeDetected),
            IntakeAuto.idleIntake(),
            new WaitCommand(1.0),
            IntakeAuto.stopIntaking()
        );
    }
}
