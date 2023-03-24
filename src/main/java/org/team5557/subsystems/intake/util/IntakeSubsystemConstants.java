package org.team5557.subsystems.intake.util;

import org.library.team3061.util.CANDeviceFinder;
import org.library.team3061.util.CANDeviceId;
import org.library.team3061.util.CANDeviceId.CANDeviceType;

public class IntakeSubsystemConstants {
    public static final CANDeviceId motorID = new CANDeviceId(CANDeviceType.SPARK_MAX, 60);
    public static final int kSolenoidForward = 9;
    public static final int kSolenoidReverse = 8;
    public static final int kProximitySwitch = 9;

    public static final String kSubsystemID = "Intake";

    public static final double kShiverTime = 0.5;
    public static final double kShiverHard = 0.8;
    public static final double kShiverSoft = 0.05;
    
}
