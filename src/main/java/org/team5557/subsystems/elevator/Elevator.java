package org.team5557.subsystems.elevator;

import org.library.team254.drivers.ServoMotorSubsystemRel;
import org.library.team254.motion.IMotionProfileGoal;
import org.library.team254.motion.MotionProfileConstraints;
import org.library.team254.motion.MotionState;
import org.library.team254.motion.SetpointGenerator.Setpoint;
import org.library.team254.util.LatchedBoolean;
import org.team5557.Constants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.planners.superstructure.util.SuperstructureConstants;
import org.team5557.subsystems.elevator.commands.HomeElevator;
import org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants;

import static org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends ServoMotorSubsystemRel {
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    private DigitalInput magLimitSwitch = new DigitalInput(kHomeLimitSwitchPort);

    private LatchedBoolean mJustReset = new LatchedBoolean();
    private MotionProfileConstraints mMotionProfileConstraintsDown;
    private IMotionProfileGoal p;

    ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.elevator_readout_key);

    public Elevator(ServoMotorSubsystemRelConstants constants) {
        super(constants);

        mHasBeenZeroed = true;
        mMotionProfileConstraints = ElevatorSubsystemConstants.motionConstraints;
        mMotionProfileConstraintsDown = ElevatorSubsystemConstants.motionConstraintsDown;

        tab.addString("Control Mode", () -> mControlState.toString());
        tab.addDouble("Height (in.)", () -> this.getPosition());
        tab.addDouble("Setpoint", () -> this.getSetpoint());
        tab.addDouble("Velocity (in./s)", () -> this.getVelocity());

        tab.addBoolean("atHomingLocation", () -> this.atHomingLocation());
        tab.addBoolean("isHomed", ()-> this.isHomed());
        tab.addBoolean("isHoming", () ->this.isHoming());

        this.outputTelemetry();

        new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
            this.setBrakeMode(IdleMode.kBrake);
        }, () -> {
            this.setBrakeMode(IdleMode.kCoast);
            }
            )
        );
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (atHomingLocation()) {//(!isHomed() && atHomingLocation()) {
            zeroSensors();
            mHasBeenZeroed = true;
        }
        this.outputTelemetry();
    }

    @Override
    public synchronized boolean atHomingLocation() {
        return !magLimitSwitch.get();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset)) {
                mHasBeenZeroed = false;
                //set arm and wrist to viable spots
                //RobotContainer.homeElevatorCommand.schedule();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();
    }

    public synchronized boolean atSetpoint() {
        return Math.abs(getPosition() - getSetpoint()) < SuperstructureConstants.kElevatorPaddingInches;
    }

    public synchronized boolean isHoming() {
        if(RobotContainer.homeElevatorCommand != null) {
            return RobotContainer.homeElevatorCommand.isScheduled();
        }
        return false;
    }

    public synchronized boolean isHomed() {
        return mHasBeenZeroed;
    }

    public synchronized void setHomed(boolean homed) {
        this.mHasBeenZeroed = homed;
    }

    public synchronized void disableSoftLimits() {
        mMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
        mMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public synchronized void enableSoftLimits() {
        mMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
        mMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public synchronized void setMotionProfilingGoal(IMotionProfileGoal goal) {
        if (mControlState != ControlState.MOTION_PROFILING_254) {
            mControlState = ControlState.MOTION_PROFILING_254;
            mMotionStateSetpoint = new MotionState(mPeriodicIO.timestamp, mPeriodicIO.position_units, 0.0, 0.0);
            mSetpointGenerator.reset();
        }
        Setpoint setpoint;
        if(goal.pos() > this.getPosition() || true) {
            setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraints, goal, mMotionStateSetpoint, mPeriodicIO.timestamp + mConstants.kLooperDt);
        } else {
            setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraintsDown, goal, mMotionStateSetpoint, mPeriodicIO.timestamp + mConstants.kLooperDt);
        }
        mPeriodicIO.demand = constrainUnits(setpoint.motion_state.pos());
        mPeriodicIO.feedforward = 0.0;//feedforward.calculate(setpoint.motion_state.vel(), setpoint.motion_state.acc());
        mMotionStateSetpoint = setpoint.motion_state;
    }
}
