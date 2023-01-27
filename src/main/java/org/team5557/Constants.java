// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode robot_mode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean tuning_mode = false;
  public static final double kloop_period = 0.02;
  public static final int klong_CAN_TimeoutMs = 100;
  public static final int kCAN_TimeoutMs = 10;

  public static class CoPilotConstants {
    public double path_regeneration_time;
    public double max_path_length;
    public double min_path_length;
    public Pose2d error_radius;
  }

  public static final CoPilotConstants copilot = new CoPilotConstants();
  static {
    copilot.path_regeneration_time = 1.0;
    copilot.max_path_length = Units.inchesToMeters(150.0);
    copilot.min_path_length = Units.inchesToMeters(12.0);
    copilot.error_radius = new Pose2d(Units.inchesToMeters(4), Units.inchesToMeters(4), Rotation2d.fromDegrees(3));
  }

  public static class SuperstructureConstants {
    public double trackwidth;
    public double drivebase;

    public Translation2d center_of_rotation;
    public Translation2d intake_center_of_rotation;
  }

  public static final SuperstructureConstants superstructure = new SuperstructureConstants();
  static {
    superstructure.trackwidth = 24.0;
    superstructure.drivebase = 24.0;

    superstructure.center_of_rotation = new Translation2d();
    superstructure.intake_center_of_rotation = new Translation2d(1.0, 0.0);
  }

  public static class PortConstants {
    public int primary_controller;
    public int pigeon;
    public int candle;
    public String canbus_name;

    public int underglow_start_index;
    public int underglow_end_index;
    public int underglow_num_leds;
  }

  public static final PortConstants ports = new PortConstants();
  static {
    ports.primary_controller = 0;
    ports.candle = 13;
    ports.pigeon = 2;
    ports.canbus_name = "canivore1";

    ports.underglow_start_index = 0;
    ports.underglow_end_index = 67;
    ports.underglow_num_leds = ports.underglow_end_index - ports.underglow_start_index;
  }

  public static class PathPlannerConstants {
    public PathConstraints fast_constraints;
    public PathConstraints medium_constraints;
    public PathConstraints slow_constraints;
    public PathConstraints hellaslow_constraints;
  }

  public static final PathPlannerConstants pathplanner = new PathPlannerConstants();
  static {
    pathplanner.fast_constraints = new PathConstraints(4.0, 3.0);
    pathplanner.medium_constraints = new PathConstraints(3.0, 1);
    pathplanner.slow_constraints = new PathConstraints(2.0, 1.0);
    pathplanner.hellaslow_constraints = new PathConstraints(1.0, 1.0);
  }

  public static class FollowerConstants {
    public double theta_kP;
    public double theta_kI;
    public double theta_kD;
    public double theta_kV;
    public double theta_kA;

    public double translation_kP;
    public double translation_kI;
    public double translation_kD;
  }

  public static final FollowerConstants follower = new FollowerConstants();
  static {
    follower.theta_kP = 6.37;
    follower.theta_kI = 0.0;
    follower.theta_kD = 0.3;
    follower.theta_kV = 2.0 * Math.PI;
    follower.theta_kA = 10.0 * Math.PI;

    follower.translation_kP = 5.0;
    follower.translation_kI = 0.0;
    follower.translation_kD = 0.0;
  }

  public static class ShuffleboardConstants {
    public String driver_readout_key;
    public String swerve_readout_key;
    public String tunable_readout_key;
    public String vision_readout_key;
    public String supervisor_readout_key;
  }

  public static final ShuffleboardConstants shuffleboard = new ShuffleboardConstants();
  static {
    shuffleboard.driver_readout_key = "Driver";
    shuffleboard.swerve_readout_key = "Swerve";
    shuffleboard.tunable_readout_key = "Tunable";
    shuffleboard.vision_readout_key = "Vision";
    shuffleboard.supervisor_readout_key = "RobotStateSupervisor";
  }

  public static class PoseEstimatorConstants {
    public Matrix<N3, N1> normalVisionStdDevs;
    public Matrix<N3, N1> highAccuracyVisionStdDevs;
    public Matrix<N3, N1> stateStdDevs;

    public double max_high_accuracy_distance;
  }

  public static final PoseEstimatorConstants estimator = new PoseEstimatorConstants();
  static {
    estimator.max_high_accuracy_distance = Units.inchesToMeters(36.0);

    estimator.highAccuracyVisionStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(3));
    estimator.normalVisionStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)); //X, Y, Theta
    estimator.stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); //X, Y, Theta
  }

  public static class RobotStateSupervisorConstants {
    public double maxSkidErrorMeters;
    public double visionToEstimatorConvergenceThreshold;
    public double visionConvergenceExpiryTime;
  }
  public static final RobotStateSupervisorConstants supervisor = new RobotStateSupervisorConstants();
  static {
    supervisor.maxSkidErrorMeters = 1.0;
    supervisor.visionToEstimatorConvergenceThreshold = 0.25;
    supervisor.visionConvergenceExpiryTime = 5.0;
  }
}
