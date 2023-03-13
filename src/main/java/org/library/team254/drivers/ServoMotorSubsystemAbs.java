package org.library.team254.drivers;

import org.library.team254.motion.IMotionProfileGoal;
import org.library.team254.motion.MotionProfileConstraints;
import org.library.team254.motion.MotionState;
import org.library.team254.motion.SetpointGenerator;
import org.library.team254.motion.SetpointGenerator.Setpoint;
import org.library.team2910.math.MathUtils;
import org.library.team3061.util.CANDeviceId;
import org.library.team3061.util.CANDeviceId.CANDeviceType;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 */
public abstract class ServoMotorSubsystemAbs extends SubsystemBase {
    private static final int kMotionProfileSlot = 0;
    private static final int kPositionPIDSlot = 1;

    // Recommend initializing in a static block!
    public static class SparkMaxConstants {
        public CANDeviceId id = new CANDeviceId(CANDeviceType.SPARK_MAX, -1);
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
        //public int encoder_ppr = 2048;
    }

    // Recommend initializing in a static block!
    public static class ServoMotorSubsystemAbsConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public double kLooperDt = 0.02;
        public int kCANTimeoutMs = 10; // use for important on the fly updates
        public int kLongCANTimeoutMs = 100; // use for constructors

        public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
        public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

        public double kRevsPerUnitDistance = 1.0;
        public double kKp = 0;  // Raw output / raw error
        public double kKi = 0;  // Raw output / sum of raw error
        public double kKd = 0;  // Raw output / (err - prevErr)
        public double kKf = 0;  // Raw output / velocity in ticks/100ms
        public double kKa = 0;  // Raw output / accel in (ticks/100ms) / s
        public double kMaxIntegralAccumulator = 0;
        public int kIZone = 0; // Ticks
        public int kDeadband = 0; // Ticks

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public double kPositionMaxIntegralAccumulator = 0;
        public int kPositionIZone = 0; // Ticks
        public int kPositionDeadband = 0; // Ticks

        public double kCruiseVelocity = 0; // units/s
        public double kAcceleration = 0; // units / s / s
        public TrapezoidProfile.Constraints profileConstraints = new TrapezoidProfile.Constraints(kCruiseVelocity, kAcceleration);
        public double kRampRate = 0.0; // s
        public double kMaxVoltage = 12.0;

        public int kStallCurrentLimit = 15;
        public int kFreeCurrentLimit = 30;
        public int kLimitRPM = 1000;
        public boolean kEnableCurrentLimit = false;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

        public int kStatus0FrameRate = 20;
        public int kStatus1FrameRate = 20;
        public int kStatus2FrameRate = 20;
        public int kStatus3FrameRate = 20;
        public int kStatus4FrameRate = 20;
        public int kStatus5FrameRate = 20;

        public int kStatusFrame8UpdateRate = 1000;
        public boolean kRecoverPositionOnReset = false;
    }

    protected final ServoMotorSubsystemAbsConstants mConstants;
    protected final CANSparkMax mMaster;
    protected final CANSparkMax[] mSlaves;

    protected final SparkMaxPIDController mMasterPID;
    protected final SparkMaxAbsoluteEncoder mMasterEncoder;

    protected ServoMotorSubsystemAbs(final ServoMotorSubsystemAbsConstants constants) {
        mConstants = constants;
        mMaster = SparkMaxFactory.createDefaultSparkMax(mConstants.kMasterConstants.id.getDeviceNumber());//new CANSparkMax(mConstants.kMasterConstants.id.getDeviceNumber(), MotorType.kBrushless);
        mMasterPID = mMaster.getPIDController();
        mMasterEncoder = mMaster.getAbsoluteEncoder(Type.kDutyCycle);

        mSlaves = new CANSparkMax[mConstants.kSlaveConstants.length];

        mMaster.setCANTimeout(mConstants.kLongCANTimeoutMs);

        SparkMaxUtil.checkError(
            mMasterPID.setFeedbackDevice(mMasterEncoder), 
                mConstants.kName + ": Could not detect encoder: ");
        
        SparkMaxUtil.checkError(mMasterEncoder.setPositionConversionFactor(
            mConstants.kRevsPerUnitDistance), 
                getName());

        SparkMaxUtil.checkError(mMasterEncoder.setVelocityConversionFactor(
            mConstants.kRevsPerUnitDistance / 60.0), 
                getName());

        /////////Setting Limits\\\\\\\\\\\
        SparkMaxUtil.checkError(
            mMaster.setSoftLimit(SoftLimitDirection.kForward, (float) mConstants.kMaxUnitsLimit),
                mConstants.kName + ": Could not set forward soft limit: ");

        SparkMaxUtil.checkError(
            mMaster.enableSoftLimit(SoftLimitDirection.kForward, true),
                mConstants.kName + ": Could not enable forward soft limit: ");

        SparkMaxUtil.checkError(
            mMaster.setSoftLimit(SoftLimitDirection.kReverse, (float) mConstants.kMinUnitsLimit),
                mConstants.kName + ": Could not set reverse soft limit: ");

        SparkMaxUtil.checkError(
            mMaster.enableSoftLimit(SoftLimitDirection.kReverse, true),
                mConstants.kName + ": Could not enable reverse soft limit: ");

        /////////Setting SmartMotion PID\\\\\\\
        SparkMaxUtil.checkError(
            mMasterPID.setP(mConstants.kKp, kMotionProfileSlot),
                mConstants.kName + ": could not set kP: ");

        SparkMaxUtil.checkError(
            mMasterPID.setI(mConstants.kKi, kMotionProfileSlot),
                mConstants.kName + ": could not set kI: ");

        SparkMaxUtil.checkError(
            mMasterPID.setD(mConstants.kKd, kMotionProfileSlot),
                mConstants.kName + ": could not set kD: ");

        SparkMaxUtil.checkError(
            mMasterPID.setFF(mConstants.kKf, kMotionProfileSlot),
                mConstants.kName + ": could not set kFF: ");

        SparkMaxUtil.checkError(
            mMasterPID.setIMaxAccum(mConstants.kMaxIntegralAccumulator, kMotionProfileSlot),
                mConstants.kName + ": Could not set max integral: ");

        SparkMaxUtil.checkError(
            mMasterPID.setIZone(mConstants.kIZone, kMotionProfileSlot),
                mConstants.kName + ": Could not set i zone: ");

        SparkMaxUtil.checkError(
            mMasterPID.setSmartMotionAllowedClosedLoopError(mConstants.kDeadband, kMotionProfileSlot),
                mConstants.kName + ": Could not set deadband: ");

        SparkMaxUtil.checkError(
            mMasterPID.setSmartMotionMaxVelocity(mConstants.kCruiseVelocity, kMotionProfileSlot),
                mConstants.kName + ": Could not set cruise velocity: ");
                
        SparkMaxUtil.checkError(
            mMasterPID.setSmartMotionMaxAccel(mConstants.kAcceleration, kMotionProfileSlot),
                mConstants.kName + ": Could not set acceleration: ");


        ////////Setting Position PID\\\\\\\\\
        SparkMaxUtil.checkError(
            mMasterPID.setP(mConstants.kPositionKp, kPositionPIDSlot),
                mConstants.kName + ": could not set kP: ");

        SparkMaxUtil.checkError(
            mMasterPID.setI(mConstants.kPositionKi, kPositionPIDSlot),
                mConstants.kName + ": could not set kI: ");

        SparkMaxUtil.checkError(
            mMasterPID.setD(mConstants.kPositionKd, kPositionPIDSlot),
                mConstants.kName + ": could not set kD: ");

        SparkMaxUtil.checkError(
            mMasterPID.setFF(mConstants.kPositionKf, kPositionPIDSlot),
                mConstants.kName + ": could not set kFF: ");

        SparkMaxUtil.checkError(
            mMasterPID.setIMaxAccum(mConstants.kMaxIntegralAccumulator, kPositionPIDSlot),
                mConstants.kName + ": Could not set max integral: ");

        SparkMaxUtil.checkError(
            mMasterPID.setIZone(mConstants.kIZone, kPositionPIDSlot),
                mConstants.kName + ": Could not set i zone: ");

        SparkMaxUtil.checkError(
            mMasterPID.setIZone(mConstants.kIZone, kPositionPIDSlot),
                mConstants.kName + ": Could not set i zone: ");

        SparkMaxUtil.checkError(
            mMasterPID.setSmartMotionAllowedClosedLoopError(mConstants.kDeadband, kPositionPIDSlot),
                mConstants.kName + ": Could not set deadband: ");

        
        ////////////Voltage, Current, Stuff\\\\\\\\\\\\\

        SparkMaxUtil.checkError(
            mMaster.setClosedLoopRampRate(mConstants.kRampRate),
                mConstants.kName + ": Could not set voltage ramp rate: ");

        SparkMaxUtil.checkError(
            mMaster.setClosedLoopRampRate(mConstants.kRampRate),
                mConstants.kName + ": Could not set closed loop ramp rate: ");

        if(mConstants.kEnableCurrentLimit) {
            SparkMaxUtil.checkError(
                mMaster.setSmartCurrentLimit(mConstants.kStallCurrentLimit, mConstants.kFreeCurrentLimit, mConstants.kLimitRPM),
                    mConstants.kName + ": Could not set supply current limit.");
        } else {
            SparkMaxUtil.checkError(
                mMaster.setSmartCurrentLimit(50),
                    mConstants.kName + ": Could not set supply current limit.");
        }

        SparkMaxUtil.checkError(
            mMaster.enableVoltageCompensation(mConstants.kMaxVoltage),
                mConstants.kName + ": Could not set voltage compensation saturation: ");

        mMasterEncoder.setAverageDepth(8);


        mMaster.setInverted(mConstants.kMasterConstants.invert_motor);
        mMasterEncoder.setInverted(mConstants.kMasterConstants.invert_motor);
        mMaster.setIdleMode(IdleMode.kBrake);

        mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, mConstants.kStatus0FrameRate);
        mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, mConstants.kStatus1FrameRate);
        mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, mConstants.kStatus2FrameRate);
        mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus3, mConstants.kStatus3FrameRate);
        mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus4, mConstants.kStatus4FrameRate);
        mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus5, mConstants.kStatus5FrameRate);


        for (int i = 0; i < mSlaves.length; ++i) {
            mSlaves[i] = SparkMaxFactory.createPermanentSlaveSparkMax(mConstants.kSlaveConstants[i].id.getDeviceNumber(), mMaster);
            mSlaves[i].setInverted(mConstants.kSlaveConstants[i].invert_motor);
            mSlaves[i].setIdleMode(IdleMode.kBrake);
            mSlaves[i].follow(mMaster);
        }

        mMaster.setCANTimeout(mConstants.kCANTimeoutMs);

        stop();
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double position_units;
        public double velocity_units_per_s;
        public double output_percent;
        public double output_voltage;
        public double master_current;
        public double error_units;

        public boolean reset_occured;

        // OUTPUTS
        public double demand;
        public double feedforward;
    }

    protected enum ControlState {
        OPEN_LOOP, SMART_MOTION, POSITION_PID, MOTION_PROFILING_WPI, MOTION_PROFILING_254, MOTION_PROFILING_SCURVE
    }

    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    protected ControlState mControlState = ControlState.OPEN_LOOP;
    protected boolean mHasBeenZeroed = false;
    protected SetpointGenerator mSetpointGenerator = new SetpointGenerator();
    protected MotionProfileConstraints mMotionProfileConstraints;
    protected MotionState mMotionStateSetpoint = null;
    protected TrapezoidProfile.State mPreviousProfiledState = null;

    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mMaster.getFault(FaultID.kHasReset)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Reset! ", false);
            mPeriodicIO.reset_occured = true;
            return;
        } else {
            mPeriodicIO.reset_occured = false;
        }

        if (mMaster.getStickyFault(FaultID.kBrownout)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kBrownout.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kOvercurrent)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kOvercurrent.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kMotorFault)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kMotorFault.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kSoftLimitFwd)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kSoftLimitFwd.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kSoftLimitRev)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kSoftLimitRev.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kSensorFault)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kSensorFault.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kCANRX)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kCANRX.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kCANTX)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kCANTX.toString(), false);
            mMaster.clearFaults();
        }
        if (mMaster.getStickyFault(FaultID.kDRVFault)) {
            DriverStation.reportError(mConstants.kName + ": SparkMax Fault! " + FaultID.kDRVFault.toString(), false);
            mMaster.clearFaults();
        }
        
        mPeriodicIO.master_current = mMaster.getOutputCurrent();
        mPeriodicIO.output_voltage = mMaster.getAppliedOutput() * mMaster.getBusVoltage();
        mPeriodicIO.output_percent = mMaster.getAppliedOutput();
        mPeriodicIO.position_units = mMasterEncoder.getPosition();
        mPeriodicIO.velocity_units_per_s = mMasterEncoder.getVelocity();

        if (mControlState != ControlState.OPEN_LOOP) {
            mPeriodicIO.error_units = mPeriodicIO.position_units - mPeriodicIO.demand;
        } else {
            mPeriodicIO.error_units = 0;
        }
    }

    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.SMART_MOTION) {
            mMasterPID.setReference(mPeriodicIO.demand, ControlType.kSmartMotion, kMotionProfileSlot, mPeriodicIO.feedforward, ArbFFUnits.kVoltage);
        } else if (mControlState == ControlState.POSITION_PID || mControlState == ControlState.MOTION_PROFILING_WPI || mControlState == ControlState.MOTION_PROFILING_254) {
            mMasterPID.setReference(mPeriodicIO.demand, ControlType.kPosition, kPositionPIDSlot, mPeriodicIO.feedforward, ArbFFUnits.kVoltage);
        } else {
            mMasterPID.setReference(mPeriodicIO.demand, ControlType.kDutyCycle, kPositionPIDSlot, mPeriodicIO.feedforward, ArbFFUnits.kVoltage);
        }
    }

    public synchronized void handleMasterReset(boolean reset) {}

    @Override
    public void periodic() {
        readPeriodicInputs();
        if (mPeriodicIO.reset_occured) {
            System.out.println(mConstants.kName + ": Master SparkMax reset occurred; resetting frame rates.");
            mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, mConstants.kStatus0FrameRate);
            mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, mConstants.kStatus1FrameRate);
            mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, mConstants.kStatus2FrameRate);
            mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus3, mConstants.kStatus3FrameRate);
            mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus4, mConstants.kStatus4FrameRate);
            mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus5, mConstants.kStatus5FrameRate);
        }
        handleMasterReset(mPeriodicIO.reset_occured);
        for (CANSparkMax slave : mSlaves) {
            if (slave.getFault(FaultID.kHasReset)) {
                System.out.println(mConstants.kName + ": Slave SparkMax reset occurred");
            }
        }
        writePeriodicOutputs();
        outputTelemetry();
    }

    // In "Units"
    public synchronized double getPosition() {
        return mPeriodicIO.position_units;
    }

    // In "Units per second"
    public synchronized double getVelocity() {
        return mPeriodicIO.velocity_units_per_s;
    }

    public synchronized double getSetpoint() {
        return (mControlState == ControlState.SMART_MOTION ||
                mControlState == ControlState.POSITION_PID ||
                mControlState == ControlState.MOTION_PROFILING_254||
                mControlState == ControlState.MOTION_PROFILING_WPI||
                mControlState == ControlState.MOTION_PROFILING_SCURVE) ?
                mPeriodicIO.demand : Double.NaN;
    }

    public synchronized void setSetpointSmartMotion(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainUnits(units);
        mPeriodicIO.feedforward = feedforward_v;
        if (mControlState != ControlState.SMART_MOTION) {
            mControlState = ControlState.SMART_MOTION;
        }
    }

    public synchronized void setSetpointSmartMotion(double units) {
        setSetpointSmartMotion(units, 0.0);
    }

    public synchronized void setMotionProfilingGoal(TrapezoidProfile.State goal, double feedforward_v) {
        if (mControlState != ControlState.MOTION_PROFILING_WPI || mPreviousProfiledState == null) {
            mControlState = ControlState.MOTION_PROFILING_WPI;
            mPreviousProfiledState = new TrapezoidProfile.State(mPeriodicIO.position_units, 0.0);//mPeriodicIO.velocity_units_per_s);
        }
        TrapezoidProfile profile = new TrapezoidProfile(mConstants.profileConstraints, goal, mPreviousProfiledState);
        mPreviousProfiledState = profile.calculate(mConstants.kLooperDt);
        mPeriodicIO.demand = constrainUnits(mPreviousProfiledState.position);
        mPeriodicIO.feedforward = feedforward_v;
    }

    public synchronized void setMotionProfilingGoal(IMotionProfileGoal goal, double feedforward_v) {
        if (mControlState != ControlState.MOTION_PROFILING_254) {
            mControlState = ControlState.MOTION_PROFILING_254;
            mMotionStateSetpoint = new MotionState(mPeriodicIO.timestamp, mPeriodicIO.position_units, mPeriodicIO.velocity_units_per_s, 0.0);
            mSetpointGenerator.reset();
        }
        Setpoint setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraints, goal, mMotionStateSetpoint, mPeriodicIO.timestamp + mConstants.kLooperDt);
        mPeriodicIO.demand = constrainUnits(setpoint.motion_state.pos());
        mPeriodicIO.feedforward = feedforward_v;
        mMotionStateSetpoint = setpoint.motion_state;
    }

    public synchronized void setSetpointPositionPID(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainUnits(units);
        mPeriodicIO.feedforward = feedforward_v;
        if (mControlState != ControlState.POSITION_PID) {
            mControlState = ControlState.POSITION_PID;
        }
    }

    public synchronized void setSetpointPositionPID(double units) {
        setSetpointPositionPID(units, 0.0);
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
    }

    public void stop() {
        setOpenLoop(0.0);
        mMaster.set(0.0);
    }

    protected double constrainUnits(double units) {
        return MathUtils.clamp(units, mConstants.kMinUnitsLimit, mConstants.kMaxUnitsLimit);
    }

    public synchronized double getPredictedPositionUnits(double lookahead_secs) {
        double predicted_units = mPeriodicIO.position_units +
                lookahead_secs * mPeriodicIO.velocity_units_per_s;
                //+ 0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);

        return predicted_units;
    }

    public void outputTelemetry() {
        Logger.getInstance().recordOutput(mConstants.kName + "/Demand", mPeriodicIO.demand);
        Logger.getInstance().recordOutput(mConstants.kName + "/PositionDeg", mPeriodicIO.position_units);
        Logger.getInstance().recordOutput(mConstants.kName + "/PositionError", mPeriodicIO.error_units);
        Logger.getInstance().recordOutput(mConstants.kName + "/VelocityDegPerSec", mPeriodicIO.velocity_units_per_s);
        Logger.getInstance().recordOutput(mConstants.kName + "/OutputDutyCycle", mPeriodicIO.output_percent);
        Logger.getInstance().recordOutput(mConstants.kName + "/OutputVoltage", mPeriodicIO.output_voltage);
        Logger.getInstance().recordOutput(mConstants.kName + "/OutputCurrent", mPeriodicIO.master_current);
        Logger.getInstance().recordOutput(mConstants.kName + "/FeedForwardVolts", mPeriodicIO.feedforward);
    }
}