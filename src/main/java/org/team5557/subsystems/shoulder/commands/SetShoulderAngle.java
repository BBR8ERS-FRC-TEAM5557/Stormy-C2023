package org.team5557.subsystems.shoulder.commands;

import org.library.team254.motion.IMotionProfileGoal;
import org.library.team254.motion.MotionProfileGoal;
import org.library.team254.motion.MotionState;
import org.team5557.RobotContainer;
import org.team5557.subsystems.shoulder.Shoulder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShoulderAngle extends CommandBase {
    Shoulder shoulder = RobotContainer.shoulder;
    double setpoint;

    public SetShoulderAngle(double setpoint) {
        this.setpoint = setpoint;
        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        //shoulder.setSetpointPositionPID(setpoint);
        //TrapezoidProfile.State state = new TrapezoidProfile.State();
        MotionProfileGoal state = new MotionProfileGoal(setpoint);
        shoulder.setMotionProfilingGoal(state, 0.0);
    }
    
}