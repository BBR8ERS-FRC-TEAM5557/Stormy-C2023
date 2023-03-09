package org.team5557.subsystems.wrist.util;

import org.library.team254.drivers.ServoMotorSubsystemAbs.ServoMotorSubsystemAbsConstants;
import org.library.team254.drivers.ServoMotorSubsystemAbs.SparkMaxConstants;
import org.library.team3061.util.CANDeviceId;
import org.library.team3061.util.CANDeviceId.CANDeviceType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristSubsystemConstants {
    public static final ServoMotorSubsystemAbsConstants kWristConstants = new ServoMotorSubsystemAbsConstants();
    static {
        kWristConstants.kName = "Wrist";
    
        kWristConstants.kLooperDt = 0.02;
        kWristConstants.kCANTimeoutMs = 10; // use for important on the fly updates
        kWristConstants.kLongCANTimeoutMs = 100; // use for constructors
    
        kWristConstants.kMasterConstants = new SparkMaxConstants();
        kWristConstants.kMasterConstants.id = new CANDeviceId(CANDeviceType.SPARK_MAX, 41);
        kWristConstants.kMasterConstants.invert_motor = false;
    
        kWristConstants.kRevsPerUnitDistance = 1.0;
        kWristConstants.kKp = 0.001;  // Raw output / raw error
        kWristConstants.kKi = 0;  // Raw output / sum of raw error
        kWristConstants.kKd = 0;  // Raw output / (err - prevErr)
        kWristConstants.kKf = 0;  // Raw output / velocity in ticks/100ms
        kWristConstants.kKa = 0;  // Raw output / accel in (ticks/100ms) / s
            kWristConstants.kMaxIntegralAccumulator = 0;
            kWristConstants.kIZone = 0; // Ticks
            kWristConstants.kDeadband = 0; // Ticks
    
            kWristConstants.kPositionKp = 0.001;
            kWristConstants.kPositionKi = 0;
            kWristConstants.kPositionKd = 0;
            kWristConstants.kPositionKf = 0;
            kWristConstants.kPositionMaxIntegralAccumulator = 0;
            kWristConstants.kPositionIZone = 0; // Ticks
            kWristConstants.kPositionDeadband = 0; // Ticks
    
            kWristConstants.kCruiseVelocity = 180.0; // units/s
            kWristConstants.kAcceleration = Math.pow(kWristConstants.kCruiseVelocity, 3); // units / s / s
            kWristConstants.profileConstraints = new TrapezoidProfile.Constraints(kWristConstants.kCruiseVelocity, kWristConstants.kAcceleration);
            kWristConstants.kRampRate = 0.0; // s
            kWristConstants.kMaxVoltage = 12.0;
    
            kWristConstants.kStallCurrentLimit = 15;
            kWristConstants.kFreeCurrentLimit = 30;
            kWristConstants.kLimitRPM = 1000;
            kWristConstants.kEnableCurrentLimit = false;
    
            kWristConstants.kMaxUnitsLimit = 359.0;
            kWristConstants.kMinUnitsLimit = 1.0;
    
            kWristConstants.kStatus0FrameRate = 20;
            kWristConstants.kStatus1FrameRate = 20;
            kWristConstants.kStatus2FrameRate = 20;
            kWristConstants.kStatus3FrameRate = 20;
            kWristConstants.kStatus4FrameRate = 20;
            kWristConstants.kStatus5FrameRate = 20;
    }
}
