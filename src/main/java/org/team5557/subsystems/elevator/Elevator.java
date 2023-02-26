package org.team5557.subsystems.elevator;

import org.library.team254.drivers.ServoMotorSubsystemRel;
import org.library.team254.util.LatchedBoolean;
import org.team5557.Constants;
import org.team5557.subsystems.elevator.commands.HomeElevator;
import static org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants.*;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends ServoMotorSubsystemRel {
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    private DigitalInput magLimitSwitch = new DigitalInput(Constants.ports.elevatorLimitSwitch);

    private boolean mHomed = false;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private Command mHomeElevatorCommand = new HomeElevator();

    ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.elevator_readout_key);

    public Elevator(ServoMotorSubsystemRelConstants constants) {
        super(constants);

        tab.addString("Control Mode", () -> mControlState.toString());
        tab.addDouble("Height (in.)", this::getPosition);
        tab.addDouble("Setpoint", this::getSetpoint);
        tab.addDouble("Velocity (in./s)", this::getVelocity);

        tab.addBoolean("atHomingLocation", this::atHomingLocation);
        tab.addBoolean("isHomed", this::isHomed);
        tab.addBoolean("isHoming", this::isHoming);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

    }

    @Override
    public void periodic() {
        super.periodic();

        if (!isHomed() && atHomingLocation()) {
            zeroSensors();
            mHomed = true;
        }
    }

    @Override
    public synchronized boolean atHomingLocation() {
        return magLimitSwitch.get();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset)) {
                mHomed = false;
                //set arm and wrist to viable spots
                scheduleHomingCommand();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();
    }

    public synchronized void scheduleHomingCommand() {
        mHomeElevatorCommand.schedule();
    }

    public synchronized boolean isHoming() {
        return mHomeElevatorCommand.isScheduled();
    }

    public synchronized boolean isHomed() {
        return mHomed;
    }

    public synchronized void disableSoftLimits() {
        mMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
        mMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public synchronized void enableSoftLimits() {
        mMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
        mMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    
    //motion profiling with automatic feedforward
    public synchronized void setMotionProfilingGoal(TrapezoidProfile.State goal) {
        if (mControlState != ControlState.MOTION_PROFILING_WPI) {
            mControlState = ControlState.MOTION_PROFILING_WPI;
            mPreviousProfiledState = new TrapezoidProfile.State(mPeriodicIO.position_units, mPeriodicIO.velocity_units_per_s);
        }
        TrapezoidProfile profile = new TrapezoidProfile(mConstants.profileConstraints, goal, mPreviousProfiledState);
        mPreviousProfiledState = profile.calculate(mConstants.kLooperDt);
        mPeriodicIO.demand = constrainUnits(mPreviousProfiledState.position);
        mPeriodicIO.feedforward = feedforward.calculate(goal.velocity);
    }
}
