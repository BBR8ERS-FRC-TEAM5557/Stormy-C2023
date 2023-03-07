package org.team5557.subsystems.elevator;

import org.library.team254.drivers.ServoMotorSubsystemRel;
import org.library.team254.util.LatchedBoolean;
import org.team5557.Constants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
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

    private LatchedBoolean mJustReset = new LatchedBoolean();

    ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.elevator_readout_key);

    public Elevator(ServoMotorSubsystemRelConstants constants) {
        super(constants);

        mHasBeenZeroed = true;

        tab.addString("Control Mode", () -> mControlState.toString());
        tab.addDouble("Height (in.)", () -> this.getPosition());
        tab.addDouble("Setpoint", () -> this.getSetpoint());
        tab.addDouble("Velocity (in./s)", () -> this.getVelocity());

        tab.addBoolean("atHomingLocation", () -> this.atHomingLocation());
        tab.addBoolean("isHomed", ()-> this.isHomed());
        tab.addBoolean("isHoming", () ->this.isHoming());

        this.outputTelemetry();
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
                RobotContainer.homeElevatorCommand.schedule();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        super.writePeriodicOutputs();
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
    
    //motion profiling with automatic feedforward
    public synchronized void setMotionProfilingGoal(double position) {
        if (mControlState != ControlState.MOTION_PROFILING_WPI) {
            mControlState = ControlState.MOTION_PROFILING_WPI;
            mPreviousProfiledState = new TrapezoidProfile.State(mPeriodicIO.position_units, mPeriodicIO.velocity_units_per_s);
        }
        TrapezoidProfile.State goal = new TrapezoidProfile.State(position, 0.0);

        TrapezoidProfile profile = new TrapezoidProfile(mConstants.profileConstraints, goal, mPreviousProfiledState);
        mPreviousProfiledState = profile.calculate(mConstants.kLooperDt);
        mPeriodicIO.demand = constrainUnits(mPreviousProfiledState.position);
        mPeriodicIO.feedforward = feedforward.calculate(goal.velocity);
    }
}
