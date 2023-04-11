package org.team5557.subsystems.swerve.util;

import org.library.team4481.SecondOrderSwerveKinematics;
import org.team5557.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public final class SwerveSubsystemConstants {
        public static final int FL_DRIVE_MOTOR = 20;
        public static final int FL_STEER_MOTOR = 24;
        public static final int FL_CANCODER = 10;
        public static final double FL_OFFSET = -10.45;

        public static final int FR_DRIVE_MOTOR = 21;
        public static final int FR_STEER_MOTOR = 25;
        public static final int FR_CANCODER = 11;
        public static final double FR_OFFSET = -166.28 + 180.0;

        public static final int BL_DRIVE_MOTOR = 22;
        public static final int BL_STEER_MOTOR = 26;
        public static final int BL_CANCODER = 12;
        public static final double BL_OFFSET = -95.44 + 180.0;

        public static final int BR_DRIVE_MOTOR = 23;
        public static final int BR_STEER_MOTOR = 27;
        public static final int BR_CANCODER = 13;
        public static final double BR_OFFSET = -261.1 + 180.0;

        public static final SwerveModuleState[] X_OUT_STATES = {
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
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

        public static final SecondOrderSwerveKinematics SECOND_KINEMATICS = new SecondOrderSwerveKinematics(
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

        public static final SwerveModuleState[] DEFAULT_STATES = new SwerveModuleState[] { 
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(), 
                new SwerveModuleState() 
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
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.2;
        /*
         * 5678.0
         * / 60.0
         * / SwerveModuleConstants.DRIVE_GEAR_RATIO
         * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
         */

        public static final double ROTATIONAL_STATIC_CONSTANT = 0.1;

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

}