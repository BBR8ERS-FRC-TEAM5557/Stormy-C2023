package org.team5557.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;

/** Swerve module hardware abstraction interface. */
public interface SwerveModuleIO {
  
  @AutoLog
  public static class SwerveModuleIOInputs {
    double drivePositionDeg = 0.0;
    double driveDistanceMeters = 0.0;
    double driveVelocityMetersPerSec = 0.0;
    double driveAppliedPercentage = 0.0;
    double[] driveCurrentAmps = new double[] {};
    double[] driveTempCelcius = new double[] {};

    double angleAbsolutePositionDeg = 0.0;
    double anglePositionRad = 0.0;
    double angleVelocityRadPerSec = 0.0;
    double angleAppliedPercentage = 0.0;
    double[] angleCurrentAmps = new double[] {};
    double[] angleTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified percentage of full power. */
  public default void setDriveMotorPercentage(double percentage) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity) {}

  /** Run the turn motor to the specified angle. */
  public default void setAnglePosition(double degrees) {}

    /** Run the turn motor to the specified angle. */
    public default void setAnglePosition(double degrees, double turnSpeed) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setAngleBrakeMode(boolean enable) {}
}