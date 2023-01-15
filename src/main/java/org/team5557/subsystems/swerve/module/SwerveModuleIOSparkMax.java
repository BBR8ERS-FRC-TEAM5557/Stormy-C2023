package org.team5557.subsystems.swerve.module;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import static org.team5557.subsystems.swerve.module.SwerveModuleConstants.*;

import org.library.team254.drivers.SparkMaxUtil;
import org.library.team3061.util.CANDeviceFinder;
import org.library.team3061.util.CANDeviceId.CANDeviceType;
import org.library.team6328.util.TunableNumber;
import org.team5557.Constants;
import org.team5557.subsystems.swerve.Conversions;


public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final TunableNumber driveKp = new TunableNumber("Drive/DriveKp", DRIVE_KP);
    private final TunableNumber driveKi = new TunableNumber("Drive/DriveKi", DRIVE_KI);
    private final TunableNumber driveKd = new TunableNumber("Drive/DriveKd", DRIVE_KD);
    private final TunableNumber turnKp = new TunableNumber("Drive/TurnKp", ANGLE_KP);
    private final TunableNumber turnKi = new TunableNumber("Drive/TurnKi", ANGLE_KI);
    private final TunableNumber turnKd = new TunableNumber("Drive/TurnKd", ANGLE_KD);

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private SparkMaxPIDController mAngleMotorPID;
    private SparkMaxPIDController mDriveMotorPID;
    private RelativeEncoder mAngleMotorEncoder;
    private RelativeEncoder mDriveMotorEncoder;
    private CANCoder angleEncoder;
    private SimpleMotorFeedforward feedForward;

    private double angleOffsetDeg;
    private double resetIteration = 0;

    /**
     * Make a new SwerveModuleIOTalonFX object.
     *
     * @param moduleNumber   the module number (0-3); primarily used for logging
     * @param driveMotorID   the CAN ID of the drive motor
     * @param angleMotorID   the CAN ID of the angle motor
     * @param canCoderID     the CAN ID of the CANcoder
     * @param angleOffsetDeg the absolute offset of the angle encoder in degrees
     */
    public SwerveModuleIOSparkMax(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID,
            double angleOffsetDeg) {

        this.angleOffsetDeg = angleOffsetDeg;
        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

        CANDeviceFinder can = new CANDeviceFinder();
        can.isDevicePresent(CANDeviceType.SPARK_MAX, driveMotorID, "Mod " + moduleNumber + "Drive");
        can.isDevicePresent(CANDeviceType.SPARK_MAX, angleMotorID, "Mod " + moduleNumber + "Angle");
        // check for the CANcoder on the CAN bus when supported by CANDeviceFinder

        configAngleEncoder(canCoderID);
        configAngleMotor(angleMotorID);
        configDriveMotor(driveMotorID);
    }

    private void configAngleEncoder(int canCoderID) {
        angleEncoder = new CANCoder(canCoderID);

        angleEncoder.configFactoryDefault();

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = CAN_CODER_INVERTED;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = this.angleOffsetDeg;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        angleEncoder.configAllSettings(config);
    }

    private void configAngleMotor(int angleMotorID) {
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        mAngleMotorPID = mAngleMotor.getPIDController();
        mAngleMotorEncoder = mAngleMotor.getEncoder();

        SparkMaxUtil.checkError(mAngleMotor.setSmartCurrentLimit(ANGLE_CONTINUOUS_CURRENT_LIMIT),
                "Motor ID " + angleMotorID + ": failed to set current limit");
        SparkMaxUtil.checkError(mAngleMotor.setIdleMode(IdleMode.kCoast),
                "Motor ID " + angleMotorID + ": failed to set Idle Mode");
        SparkMaxUtil.checkError(mAngleMotor.setCANTimeout(Constants.kCAN_TimeoutMs),
                "Motor ID " + angleMotorID + ": failed to set CAN timeout");

        SparkMaxUtil.checkError(mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
                "Motor ID " + angleMotorID + ": failed to set periodic status frame 0 rate");
        SparkMaxUtil.checkError(mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
                "Motor ID " + angleMotorID + ": failed to set periodic status frame 1 rate");
        SparkMaxUtil.checkError(mAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20),
                "Motor ID " + angleMotorID + ": failed to set periodic status frame 2 rate");
        mAngleMotor.setInverted(ANGLE_MOTOR_INVERTED);
        // angleMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 20;
        // SparkMaxUtil.checkError(mAngleMotor.getEncoder().setPositionConversionFactor(360
        // / ANGLE_GEAR_RATIO), null);

        SparkMaxUtil.checkError(mAngleMotorPID.setP(turnKp.get(), SLOT_INDEX), "Motor ID " + angleMotorID + ": failed to set P");
        SparkMaxUtil.checkError(mAngleMotorPID.setI(turnKi.get(), SLOT_INDEX), "Motor ID " + angleMotorID + ": failed to set I");
        SparkMaxUtil.checkError(mAngleMotorPID.setD(turnKd.get(), SLOT_INDEX), "Motor ID " + angleMotorID + ": failed to set D");
        SparkMaxUtil.checkError(mAngleMotorPID.setFF(ANGLE_KF, SLOT_INDEX), "Motor ID " + angleMotorID + ": failed to set FF");
        SparkMaxUtil.checkError(mAngleMotorPID.setFeedbackDevice(mAngleMotorEncoder), "Motor ID " + angleMotorID + ": failed to set NEO PID feedback device");

        SparkMaxUtil.checkError(mAngleMotorEncoder.setPositionConversionFactor(2.0 * Math.PI / ANGLE_GEAR_RATIO),
                "Failed to set NEO encoder conversion factor");
        SparkMaxUtil.checkError(mAngleMotorEncoder.setVelocityConversionFactor(2.0 * Math.PI / ANGLE_GEAR_RATIO / 60.0),
                "Failed to set NEO encoder conversion factor");

        double absolutePosition = getCanCoder().getRadians();// getCanCoder().getRadians() -
                                                             // Units.degreesToRadians(angleOffsetDeg);
        SparkMaxUtil.checkError(mAngleMotor.getEncoder().setPosition(absolutePosition),
                "Motor ID " + angleMotorID + ": failed to set encoder offset");
        SparkMaxUtil.checkError(mAngleMotor.burnFlash(), "Motor ID " + angleMotorID + ": failed to set burn flash");
    }

    private void configDriveMotor(int driveMotorID) {
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mDriveMotorPID = mDriveMotor.getPIDController();
        mDriveMotorEncoder = mDriveMotor.getEncoder();

        SparkMaxUtil.checkError(mAngleMotor.setSmartCurrentLimit(DRIVE_CONTINUOUS_CURRENT_LIMIT),
                "Motor ID " + driveMotorID + ": failed to set current limit");
        SparkMaxUtil.checkError(mAngleMotor.setIdleMode(IdleMode.kCoast),
                "Motor ID " + driveMotorID + ": failed to set Idle Mode");
        SparkMaxUtil.checkError(mAngleMotor.setCANTimeout(Constants.kCAN_TimeoutMs),
                "Motor ID " + driveMotorID + ": failed to set CAN timeout");

        SparkMaxUtil.checkError(mDriveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
                "Motor ID " + driveMotorID + ": failed to set periodic status frame 0 rate");
        SparkMaxUtil.checkError(mDriveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
                "Motor ID " + driveMotorID + ": failed to set periodic status frame 1 rate");
        SparkMaxUtil.checkError(mDriveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20),
                "Motor ID " + driveMotorID + ": failed to set periodic status frame 2 rate");
        // angleMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 20;
        // SparkMaxUtil.checkError(mAngleMotor.getEncoder().setPositionConversionFactor(360
        // / ANGLE_GEAR_RATIO), null);

        SparkMaxUtil.checkError(mDriveMotorPID.setP(driveKp.get(), SLOT_INDEX), "Motor ID " + driveMotorID + ": failed to set P");
        SparkMaxUtil.checkError(mDriveMotorPID.setI(driveKi.get(), SLOT_INDEX), "Motor ID " + driveMotorID + ": failed to set I");
        SparkMaxUtil.checkError(mDriveMotorPID.setD(driveKd.get(), SLOT_INDEX), "Motor ID " + driveMotorID + ": failed to set D");
        SparkMaxUtil.checkError(mDriveMotorPID.setFF(DRIVE_KF, SLOT_INDEX), "Motor ID " + driveMotorID + ": failed to set FF");

        mDriveMotorEncoder.setPosition(0);
    }

    private Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private double calculateFeedforward(double velocity) {
        double percentage = this.feedForward.calculate(velocity);
        // clamp the voltage to the maximum voltage
        if (percentage > 1.0) {
            return 1.0;
        }
        return percentage;
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionDeg = Conversions.rotationsToDegrees(
                mDriveMotorEncoder.getPosition(), SwerveModuleConstants.DRIVE_GEAR_RATIO);
        inputs.driveDistanceMeters = Conversions.rotationsToMeters(
                mDriveMotorEncoder.getPosition(),
                SwerveModuleConstants.WHEEL_CIRCUMFERENCE,
                SwerveModuleConstants.DRIVE_GEAR_RATIO);
        inputs.driveVelocityMetersPerSec = Conversions.rpmToMPS(
                mDriveMotorEncoder.getVelocity(),
                SwerveModuleConstants.WHEEL_CIRCUMFERENCE,
                SwerveModuleConstants.DRIVE_GEAR_RATIO);
        inputs.driveAppliedPercentage = mDriveMotor.getAppliedOutput();
        inputs.driveCurrentAmps = new double[] { mDriveMotor.getOutputCurrent() };
        inputs.driveTempCelcius = new double[] { mDriveMotor.getMotorTemperature() };

        
        inputs.angleAbsolutePositionDeg = angleEncoder.getAbsolutePosition();
        inputs.anglePositionRad = Conversions.convertPiPositive(mAngleMotorEncoder.getPosition());
        inputs.angleVelocityRevPerMin = mAngleMotorEncoder.getVelocity();
        inputs.angleAppliedPercentage = mAngleMotor.getAppliedOutput();
        inputs.angleCurrentAmps = new double[] { mAngleMotor.getOutputCurrent() };
        inputs.angleTempCelcius = new double[] { mAngleMotor.getMotorTemperature() };

        // update tunables
        if (driveKp.hasChanged()
                || driveKi.hasChanged()
                || driveKd.hasChanged()
                || turnKp.hasChanged()
                || turnKi.hasChanged()
                || turnKd.hasChanged()) {
            mDriveMotorPID.setP(driveKp.get(), SLOT_INDEX);
            mDriveMotorPID.setI(driveKi.get(), SLOT_INDEX);
            mDriveMotorPID.setD(driveKd.get(), SLOT_INDEX);
            mAngleMotorPID.setP(turnKp.get(), SLOT_INDEX);
            mAngleMotorPID.setI(turnKi.get(), SLOT_INDEX);
            mAngleMotorPID.setD(turnKd.get(), SLOT_INDEX);
        }
    }

    /** Run the drive motor at the specified percentage of full power. */
    @Override
    public void setDriveMotorPercentage(double percentage) {
        mDriveMotor.getPIDController().setReference(percentage, ControlType.kDutyCycle);
    }

    /** Run the drive motor at the specified velocity. */
    @Override
    public void setDriveVelocity(double velocity) {
        double rotationsPerMinute = Conversions.mpsToRPM(
                velocity,
                SwerveModuleConstants.WHEEL_CIRCUMFERENCE,
                SwerveModuleConstants.DRIVE_GEAR_RATIO);

        mDriveMotorPID.setReference(rotationsPerMinute, ControlType.kVelocity, SLOT_INDEX, calculateFeedforward(velocity),
                ArbFFUnits.kPercentOut);
    }

    /**
     * Run the turn motor to the specified angle.
     * 
     * @param referenceAngleRadians - the desired angle given in range [0,2pi]
     */
    @Override
    public void setAnglePosition(double referenceAngleRadians) {
        double currentAngleRadians = mAngleMotorEncoder.getPosition();
        if (mAngleMotorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = getCanCoder().getRadians();
                mAngleMotorEncoder.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }
        double currentAngleRadiansMod = Conversions.convertPiPositive(currentAngleRadians);
        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        // (currentAngleRadians - currentAngleRadiansMod) essentially makes the angle == to 0 pi
        // in the given full rotations order of magnitude and adding the reference angle gives the angle
        // in the proper reference frame.
        double adjustedReferenceAngleRadians = referenceAngleRadians + (currentAngleRadians - currentAngleRadiansMod);

        // if the difference between the current angle and the desired angle is greater than PI
        // rotate in the direction that makes it less than PI taking the shortest distance around
        // I'm pretty sure this does almost nothing considering the module states are already
        // optimized such that you're never rotating more PI/2
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }
        mAngleMotorPID.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        mDriveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setAngleBrakeMode(boolean enable) {
        // always leave the angle motor in coast mode
        mAngleMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

}
