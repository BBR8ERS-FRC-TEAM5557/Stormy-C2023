package org.team5557.subsystems.intake.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.Intake;
import org.team5557.subsystems.intake.util.IntakeState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAuto {

    public static Command spitCube() {
        return new InstantCommand(() -> RobotContainer.intake.setIntakeState(IntakeState.IntakeStates.EJECT_CUBE.getIntakeState()))
                    .raceWith(new WaitCommand(1.0))
                    .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeState(IntakeState.IntakeStates.DO_NOTHING.getIntakeState())));
    }

    public static Command startIntaking() {
        return new InstantCommand(() -> RobotContainer.intake.setIntakeState(IntakeState.IntakeStates.INTAKING_CUBE.getIntakeState()));
    }

    public static Command stopIntaking() {
        return new InstantCommand(() -> RobotContainer.intake.setIntakeState(IntakeState.IntakeStates.DO_NOTHING.getIntakeState()));
    }

    public static Command idleIntake() {
        return new InstantCommand(() -> RobotContainer.intake.setIntakeState(IntakeState.IntakeStates.IDLE.getIntakeState()));
    }
}
