// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import org.library.team6328.util.Alert;
import org.library.team6328.util.Alert.AlertType;
import org.team5557.auto.AutonomousChooser;
import org.team5557.auto.AutonomousTrajectories;
import org.team5557.commands.swerve.TeleopDrive;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.subsystems.swerve.RawControllers;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.SwerveSubsystemConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
  // Subsystems
<<<<<<< HEAD
  //public static final RobotStateSupervisor state_supervisor = new RobotStateSupervisor();
=======
  public static final RobotStateSupervisor state_supervisor = new RobotStateSupervisor();
>>>>>>> e65f789 (vision testing - 1/16)
  public static final Swerve swerve = new Swerve();

  // Controller
  public static final XboxController primary_controller = new XboxController(Constants.ports.primary_controller);

  // Dashboard inputs / Util
  private final AutonomousChooser autonomous_chooser = new AutonomousChooser(new AutonomousTrajectories());
  public static final RawControllers raw_controllers = new RawControllers();
    //private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Constants.tuning_mode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    //CommandScheduler.getInstance().registerSubsystem(swerve);
    //CommandScheduler.getInstance().registerSubsystem(state_supervisor);
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    swerve.setDefaultCommand(new TeleopDrive(this::getForwardInput, this::getStrafeInput, this::getRotationInput));
  }

  /**
   * Use this to pass the autonomous chooser to the main {@link Robot} class.
   *
   * @return the autonomous chooser that determines what to run in autonomous
   */
  public AutonomousChooser getAutonomousChooser() {
    return autonomous_chooser;
  }

  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance)
        return 0.0;

    return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  private static double square(double value) {
      return Math.copySign(value * value, value);
  }

  public double getForwardInput() {
      return -square(deadband(primary_controller.getLeftY(), 0.1)) * SwerveSubsystemConstants.MAX_VELOCITY_METERS_PER_SECOND;
  }

  public double getStrafeInput() {
      return -square(deadband(primary_controller.getLeftX(), 0.1)) * SwerveSubsystemConstants.MAX_VELOCITY_METERS_PER_SECOND;
  }

  public double getRotationInput() {
      return -square(deadband(primary_controller.getRightX(), 0.1)) * SwerveSubsystemConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  public Rotation2d getRightStickAngle() {
      return new Rotation2d(primary_controller.getRightX(), primary_controller.getRightY());//Math.atan2(primary_controller.getRightX(), -primary_controller.getRightY());
  }
}
