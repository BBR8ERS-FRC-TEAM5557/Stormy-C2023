package org.team5557.subsystems.swerve.module;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;

public class SwerveModuleConstants {
    private static final String CONSTRUCTOR_EXCEPTION = "constant class";

    private SwerveModuleConstants() {
        throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    /* MK4i L2 */
    public static final double MK4I_L2_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double MK4I_L2_WHEEL_CIRCUMFERENCE = MK4I_L2_WHEEL_DIAMETER_METERS * Math.PI;
    public static final double MK4I_L2_DRIVE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    public static final boolean MK4I_L2_DRIVE_MOTOR_INVERTED = true;
    public static final double MK4I_L2_ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
    public static final boolean MK4I_L2_ANGLE_MOTOR_INVERTED = true;
    public static final boolean MK4I_L2_CAN_CODER_INVERTED = false;

    // FIXME: assign these constants to the appropriate swerve module variant
    public static final double WHEEL_DIAMETER_METERS = MK4I_L2_WHEEL_DIAMETER_METERS;
    public static final double WHEEL_CIRCUMFERENCE = MK4I_L2_WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_GEAR_RATIO = MK4I_L2_DRIVE_GEAR_RATIO;
    public static final boolean DRIVE_MOTOR_INVERTED = MK4I_L2_DRIVE_MOTOR_INVERTED;
    public static final double ANGLE_GEAR_RATIO = MK4I_L2_ANGLE_GEAR_RATIO;
    public static final boolean ANGLE_MOTOR_INVERTED = MK4I_L2_ANGLE_MOTOR_INVERTED;
    public static final boolean CAN_CODER_INVERTED = MK4I_L2_CAN_CODER_INVERTED;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Swerve Current Limiting */
    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* PID SLOT INDEX */
    public static final int SLOT_INDEX = 0;

    // FIXME: tune PID values for the angle and drive motors

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 1.0;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.00002;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    // FIXME: characterize the drivetrain and update these constants

    /* Drive Motor Characterization Values */
    // divide by 12 to convert from volts to percent output for CTRE
    public static final double DRIVE_KS = (0.10095 / 12);
    public static final double DRIVE_KV = (2.55494 / 12);
    public static final double DRIVE_KA = (0.12872 / 12);

    public static final double TURN_KS = (0.15928/12);
    public static final double TURN_KV = (0.43772/12);
    public static final double TURN_KA = 0.0;

    /* Neutral Modes */
    public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kCoast;
    public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

    /* Reseting NEO Encoder */
    public static final int ENCODER_RESET_ITERATIONS = 500;
    public static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

}
