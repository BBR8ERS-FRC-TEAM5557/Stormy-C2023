package org.team5557.subsystems.swerve;

import org.team5557.Constants;
import org.team5557.subsystems.swerve.module.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SwerveSubsystemConstants {
        public static final int FL_DRIVE_MOTOR = 1;
        public static final int FL_STEER_MOTOR = 2;
        public static final int FL_CANCODER = 3;
        public static final double FL_OFFSET = 0.0;

        public static final int FR_DRIVE_MOTOR = 4;
        public static final int FR_STEER_MOTOR = 5;
        public static final int FR_CANCODER = 6;
        public static final double FR_OFFSET = 0.0;

        public static final int BL_DRIVE_MOTOR = 7;
        public static final int BL_STEER_MOTOR = 8;
        public static final int BL_CANCODER = 9;
        public static final double BL_OFFSET = 0.0;

        public static final int BR_DRIVE_MOTOR = 10;
        public static final int BR_STEER_MOTOR = 11;
        public static final int BR_CANCODER = 12;
        public static final double BR_OFFSET = 0.0;

        public static final SwerveModuleState[] X_OUT_STATES = {
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
        };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(Constants.superstructure.trackwidth / 2.0,
                                        Constants.superstructure.drivebase / 2.0),
                        // Front right
                        new Translation2d(Constants.superstructure.trackwidth / 2.0,
                                        -Constants.superstructure.drivebase / 2.0),
                        // Back left
                        new Translation2d(-Constants.superstructure.trackwidth / 2.0,
                                        Constants.superstructure.drivebase / 2.0),
                        // Back right
                        new Translation2d(-Constants.superstructure.trackwidth / 2.0,
                                        -Constants.superstructure.drivebase / 2.0));

        public static final SwerveModulePosition[] DEFAULT_POSITIONS = new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
        };

        /**
         * The formula for calculating the theoretical maximum velocity is: <Motor free
         * speed RPM> / 60 *
         * <Drive reduction> * <Wheel diameter meters> * pi By default this value is
         * setup for a Mk3
         * standard module using Falcon500s to drive.
         */

        // FIXME: determine maximum velocities empirically

        public static final double MAX_VOLTAGE = 12.0;
        /**
         * The maximum velocity of the robot in meters per second.
         *
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5678.0
                        / 60.0
                        / SwerveModuleConstants.DRIVE_GEAR_RATIO
                        * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;

        public static final double ROTATIONAL_STATIC_CONSTANT = 0.3;

        public static final double DRIVETRAIN_CURRENT_LIMIT = 50.0;

        public static final double SKID_VELOCITY_DIFFERENCE = 1.0;

        /**
         * The maximum angular velocity of the robot in radians per second.
         *
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                        / Math.hypot(Constants.superstructure.trackwidth / 2.0,
                                        Constants.superstructure.drivebase / 2.0);

        public static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

        public static final int TIMEOUT_MS = 30;

        public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
        public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2.0 * Math.PI;

        // FIXME: tune PID values for auto paths

        public static final double AUTO_DRIVE_P_CONTROLLER = 2.2956;
        public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
        public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
        public static final double AUTO_TURN_P_CONTROLLER = 4.9;
        public static final double AUTO_TURN_I_CONTROLLER = 0.0;
        public static final double AUTO_TURN_D_CONTROLLER = 0.0;

        public static final double DEADBAND = 0.1;
}