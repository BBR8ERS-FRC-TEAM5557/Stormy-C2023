package org.team5557.subsystems.elevator.util;

import org.library.team254.drivers.ServoMotorSubsystemRel.ServoMotorSubsystemRelConstants;
import org.library.team254.drivers.ServoMotorSubsystemRel.SparkMaxConstants;
import org.library.team254.motion.MotionProfileConstraints;
import org.library.team3061.util.CANDeviceId;
import org.library.team3061.util.CANDeviceId.CANDeviceType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorSubsystemConstants {
    public static double kS = 0.0;
    public static double kG = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double kHomingLocation = 0.0;
    public static int kHomeLimitSwitchPort = 1;

    public static double kMaxManualPower = 0.6;
    
    private static boolean invert_motors = true;

    public static final ServoMotorSubsystemRelConstants kElevatorConstants = new ServoMotorSubsystemRelConstants();
    static {
        kElevatorConstants.kName = "Elevator";

        kElevatorConstants.kMasterConstants.id = new CANDeviceId(CANDeviceType.SPARK_MAX, 50);
        kElevatorConstants.kMasterConstants.invert_motor = invert_motors;


        kElevatorConstants.kSlaveConstants = new SparkMaxConstants[1];

        kElevatorConstants.kSlaveConstants[0] = new SparkMaxConstants();
        kElevatorConstants.kSlaveConstants[0].id = new CANDeviceId(CANDeviceType.SPARK_MAX, 30);
        kElevatorConstants.kSlaveConstants[0].invert_motor = invert_motors;


        kElevatorConstants.kRevsPerUnitDistance = 15.0;
        kElevatorConstants.kEncoderInverted = true;

        kElevatorConstants.kKp = 0;  // Raw output / raw error
        kElevatorConstants.kKi = 0;  // Raw output / sum of raw error
        kElevatorConstants.kKd = 0;  // Raw output / (err - prevErr)
        kElevatorConstants.kKf = 0;  // Raw output / velocity in ticks/100ms
        kElevatorConstants.kKa = 0;  // Raw output / accel in (ticks/100ms) / s
        kElevatorConstants.kMaxIntegralAccumulator = 0;
        kElevatorConstants.kIZone = 0; // Ticks
        kElevatorConstants.kDeadband = 0; // units

        kElevatorConstants.kPositionKp = 0.06;
        kElevatorConstants.kPositionKi = 0;
        kElevatorConstants.kPositionKd = 0;
        kElevatorConstants.kPositionKf = 0;
        kElevatorConstants.kPositionMaxIntegralAccumulator = 0;
        kElevatorConstants.kPositionIZone = 0; //
        kElevatorConstants.kPositionDeadband = 0.5;

        kElevatorConstants.kCruiseVelocity = 95.0; // units / s
        kElevatorConstants.kAcceleration = 100.0; // inches / s / s
        kElevatorConstants.profileConstraints = new TrapezoidProfile.Constraints(kElevatorConstants.kCruiseVelocity, kElevatorConstants.kAcceleration);

        kElevatorConstants.kStallCurrentLimit = 80;
        kElevatorConstants.kFreeCurrentLimit = 80;
        kElevatorConstants.kLimitRPM = 0;
        kElevatorConstants.kEnableCurrentLimit = true;

        kElevatorConstants.kMaxUnitsLimit = 70.0;//inches
        kElevatorConstants.kMinUnitsLimit = 0.0;

        kElevatorConstants.kStatus0FrameRate = 20;
        kElevatorConstants.kStatus1FrameRate = 200;
        kElevatorConstants.kStatus2FrameRate = 200;
        kElevatorConstants.kStatus3FrameRate = 20;
        kElevatorConstants.kStatus4FrameRate = 20;
        kElevatorConstants.kStatus5FrameRate = 200;
    }

    public static MotionProfileConstraints motionConstraints = new MotionProfileConstraints(kElevatorConstants.kCruiseVelocity, -kElevatorConstants.kCruiseVelocity + 65.0, kElevatorConstants.kAcceleration);
    public static MotionProfileConstraints motionConstraintsDown = new MotionProfileConstraints(25.0, 25.0, 20.0);
}
