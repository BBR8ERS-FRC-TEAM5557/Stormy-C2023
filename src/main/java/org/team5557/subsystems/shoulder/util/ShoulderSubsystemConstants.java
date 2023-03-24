package org.team5557.subsystems.shoulder.util;

import org.library.team254.drivers.ServoMotorSubsystemAbs;
import org.library.team254.drivers.ServoMotorSubsystemAbs.ServoMotorSubsystemAbsConstants;
import org.library.team254.drivers.ServoMotorSubsystemAbs.SparkMaxConstants;
import org.library.team254.motion.MotionProfileConstraints;
import org.library.team3061.util.CANDeviceId;
import org.library.team3061.util.CANDeviceId.CANDeviceType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ShoulderSubsystemConstants {

    public static double kS = 0.0;//2.01922;
    public static double kG = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;//0.0016737;

    public static final ServoMotorSubsystemAbsConstants kShoulderConstants = new ServoMotorSubsystemAbsConstants();
    static {
            kShoulderConstants.kName = "Shoulder";
    
            kShoulderConstants.kLooperDt = 0.02;
            kShoulderConstants.kCANTimeoutMs = 10; // use for important on the fly updates
            kShoulderConstants.kLongCANTimeoutMs = 100; // use for constructors
    
            kShoulderConstants.kMasterConstants = new SparkMaxConstants();
            kShoulderConstants.kMasterConstants.id = new CANDeviceId(CANDeviceType.SPARK_MAX, 40);
            kShoulderConstants.kMasterConstants.invert_motor = true;
    
            kShoulderConstants.kRevsPerUnitDistance = 1.0;
            kShoulderConstants.kKp = 0.005;  // Raw output / raw error
            kShoulderConstants.kKi = 0;  // Raw output / sum of raw error
            kShoulderConstants.kKd = 0;  // Raw output / (err - prevErr)
            kShoulderConstants.kKf = 0;  // Raw output / velocity in ticks/100ms
            kShoulderConstants.kKa = 0;  // Raw output / accel in (ticks/100ms) / s
            kShoulderConstants.kMaxIntegralAccumulator = 0;
            kShoulderConstants.kIZone = 0; // Ticks
            kShoulderConstants.kDeadband = 0; // Ticks
    
            kShoulderConstants.kPositionKp = 0.015;
            kShoulderConstants.kPositionKi = 0;
            kShoulderConstants.kPositionKd = 0;
            kShoulderConstants.kPositionKf = 0;
            kShoulderConstants.kPositionMaxIntegralAccumulator = 0;
            kShoulderConstants.kPositionIZone = 0; // Ticks
            kShoulderConstants.kPositionDeadband = 0; // Ticks
    
            kShoulderConstants.kCruiseVelocity = 500.0; // units/s
            kShoulderConstants.kAcceleration = 500.0; // units / s / s
            kShoulderConstants.profileConstraints = new TrapezoidProfile.Constraints(kShoulderConstants.kCruiseVelocity, kShoulderConstants.kAcceleration);
            kShoulderConstants.kRampRate = 0.0; // s
            kShoulderConstants.kMaxVoltage = 12.0;
    
            kShoulderConstants.kStallCurrentLimit = 15;
            kShoulderConstants.kFreeCurrentLimit = 30;
            kShoulderConstants.kLimitRPM = 1000;
            kShoulderConstants.kEnableCurrentLimit = false;
    
            kShoulderConstants.kMaxUnitsLimit = 330.0;
            kShoulderConstants.kMinUnitsLimit = 193.0;
    
            kShoulderConstants.kStatus0FrameRate = 20;
            kShoulderConstants.kStatus1FrameRate = 200;
            kShoulderConstants.kStatus2FrameRate = 200;
            kShoulderConstants.kStatus3FrameRate = 200;
            kShoulderConstants.kStatus4FrameRate = 200;
            kShoulderConstants.kStatus5FrameRate = 20;
    }

    public static class ShoulderConstants {
        public double mass;
        public double length;
        public double moi;
        public double cgRadius;
        public double minAngle;
        public double maxAngle;
        public double reduction;
      }
      public static final ShoulderConstants kShoulderJointConstants = new ShoulderConstants();
      static {
        kShoulderJointConstants.mass = 1.0;
        kShoulderJointConstants.length = Units.inchesToMeters(12.5);
        kShoulderJointConstants.moi = 1.0;
        kShoulderJointConstants.cgRadius = Units.inchesToMeters(5.0);
        kShoulderJointConstants.minAngle = Units.degreesToRadians(90.0); //degrees
        kShoulderJointConstants.maxAngle = Units.degreesToRadians(344.0); //degrees
        kShoulderJointConstants.reduction = 108.0 / 1.0;
      }

    public static MotionProfileConstraints motionConstraints = new MotionProfileConstraints(kShoulderConstants.kCruiseVelocity, -kShoulderConstants.kCruiseVelocity, kShoulderConstants.kAcceleration);
    
}
