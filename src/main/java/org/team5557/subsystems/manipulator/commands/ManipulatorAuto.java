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

    public static Command ejectCubeMid() {
        return 
            Commands.sequence(
                startEjectingCubeMid(),
                Commands.waitSeconds(1.0),
                stopManipulator()
            );
    }

    public static Command ejectCubeHigh() {
        return 
            Commands.sequence(
                startEjectingCubeHigh(),
                Commands.waitSeconds(1.0),
                stopManipulator()
            );
    }

    public static Command ejectCone() {
        return 
            Commands.sequence(
                startEjectingCone(),
                Commands.waitSeconds(1.0),
                stopManipulator()
            );

    }

    public static Command startEjectingCube() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.EJECT_CUBE.getManipulatorState()));
    }

    public static Command startEjectingCubeMid() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.EJECT_CUBE_MID.getManipulatorState()));
    }

    public static Command startEjectingCubeHigh() {
        return new InstantCommand(() -> RobotContainer.manipulator.setManipulatorState(ManipulatorState.ManipulatorStates.EJECT_CUBE_HIGH.getManipulatorState()));
    }

    public static Command startEjectingCone() {
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
}
