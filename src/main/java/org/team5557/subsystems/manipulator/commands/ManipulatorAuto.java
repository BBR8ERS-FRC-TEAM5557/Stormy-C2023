package org.team5557.subsystems.manipulator.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.manipulator.util.ManipulatorState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ManipulatorAuto {
    
    public static Command ejectCube() {
        return 
            Commands.sequence(
                startEjectingCube(),
                Commands.waitSeconds(1.0),
                stopManipulator()
            );
    }

    public static Command fireCube() {
        return 
        Commands.sequence(
            startFiringCube(),
            Commands.waitSeconds(1.0),
            stopManipulator()
        );
    }

    public static Command ejectCone() {
        return 
            Commands.sequence(
                startEjectingCone(),
                Commands.waitSeconds(3.0),
                stopManipulator()
            );
    }

    public static Command ejectConeAuto() {
        return 
            Commands.sequence(
                startEjectingConeFast(),
                Commands.waitSeconds(1.0),
                stopManipulator()
            );
    }

    public static Command startEjectingCube() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.EJECT_CUBE.getManipulatorState()));
    }

    public static Command startFiringCube() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.FIRE_CUBE.getManipulatorState()));
    }

    public static Command startEjectingCone() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.EJECT_CONE.getManipulatorState()));
    }

    public static Command startEjectingConeFast() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.EJECT_CONE.getManipulatorState()));
    }

    public static Command stopManipulator() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.DO_NOTHING.getManipulatorState()));
    }

    public static Command startSuckingCube() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.INTAKING_CUBE.getManipulatorState()));
    }

    public static Command startSuckingCone() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.INTAKING_CONE.getManipulatorState()));
    }

    public static Command holdCube() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.HOLDING_CUBE.getManipulatorState()));
    }

    public static Command holdCone() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.HOLDING_CONE.getManipulatorState()));
    }

    public static Command suckCubeStop() {
        return 
        Commands.sequence(
            startSuckingCube(),
            waitForCube(),
            holdCube()
        );
    }

    public static Command waitForCube() {
        return Commands.waitUntil(() -> RobotContainer.manipulator.getCubeDetected());
    }
}
