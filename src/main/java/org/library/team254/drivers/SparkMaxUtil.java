package org.library.team254.drivers;

import edu.wpi.first.wpilibj.DriverStation;
import com.REVRobotics.REVLibError.*;

public class SparkMaxUtil {
    // Checks the specified error code for issues.
    public static void checkError(REVLibError errorCode, String message) {
        if (errorCode != REVLibError.kOk) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}