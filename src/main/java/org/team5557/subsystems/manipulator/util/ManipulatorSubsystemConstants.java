package org.team5557.subsystems.manipulator.util;

import org.library.team3061.util.CANDeviceId;
import org.library.team3061.util.CANDeviceId.CANDeviceType;

public class ManipulatorSubsystemConstants {
    public static final String kSubsystemID = "Manipulator";

    public static final CANDeviceId kTopRollerMotorID = new CANDeviceId(CANDeviceType.SPARK_MAX, 31); //changed for elevator testing was 50
    public static final CANDeviceId kBottomRollerMotorID = new CANDeviceId(CANDeviceType.SPARK_MAX, 51);

    public static final boolean motorsInverted = true;

    public static final int kCubeProximitySwitch = 2;
}
