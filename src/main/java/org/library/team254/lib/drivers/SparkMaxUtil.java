package org.library.team254.lib.drivers;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkMaxUtil {
    // Checks the specified error code for issues.
    public static void checkError(REVLibError errorCode, String message) {
        if (errorCode != REVLibError.kOk) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}