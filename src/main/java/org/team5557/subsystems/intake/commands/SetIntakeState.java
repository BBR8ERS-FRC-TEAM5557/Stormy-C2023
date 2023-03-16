package org.team5557.subsystems.intake.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.Intake;
import org.team5557.subsystems.intake.util.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeState extends CommandBase {
    private final Intake intake = RobotContainer.intake;
    private final IntakeState state;
    
    public SetIntakeState(IntakeState state) {
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
