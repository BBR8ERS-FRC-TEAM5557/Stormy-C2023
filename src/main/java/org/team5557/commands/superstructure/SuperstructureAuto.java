package org.team5557.commands.superstructure;

import org.team5557.RobotContainer;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.subsystems.manipulator.commands.ManipulatorAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperstructureAuto {

    private static final SuperstructureState HIGH_CONE = SuperstructureState.Preset.HIGH_CONE.getState();
    private static final SuperstructureState HIGH_CUBE = SuperstructureState.Preset.HIGH_CUBE.getState();
    private static final SuperstructureState IDLE = SuperstructureState.Preset.HOLDING.getState();

    public static Command scoreHighCone() {
        return Commands.sequence(
            new SetSuperstructureSetpoint(HIGH_CONE).until(() -> RobotContainer.state_supervisor.isAtDesiredState(HIGH_CONE)).withTimeout(5.0),
            ManipulatorAuto.ejectConeAuto(),
            new SetSuperstructureSetpoint(IDLE).until(() -> RobotContainer.state_supervisor.isElevatorBelowThreshould(0.0, 15.0))
        );
    }

    public static Command scoreHighCube() {
        return Commands.sequence(
            new SetSuperstructureSetpoint(HIGH_CUBE).until(() -> RobotContainer.state_supervisor.isAtDesiredState(HIGH_CUBE)).withTimeout(5.0),
            ManipulatorAuto.ejectCube(),
            new SetSuperstructureSetpoint(IDLE).until(() -> RobotContainer.state_supervisor.isElevatorBelowThreshould(0.0, 12.0))
        );
    }

    
}
