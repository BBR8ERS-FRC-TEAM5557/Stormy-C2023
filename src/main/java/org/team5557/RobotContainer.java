// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import java.util.List;

import org.library.team6328.util.Alert;
import org.library.team6328.util.Alert.AlertType;
import org.team5557.auto.AutonomousChooser;
import org.team5557.auto.AutonomousTrajectories;
import org.team5557.commands.superstructure.SetSuperstructureSetpoint;
import org.team5557.paths.Pathweaver;
import org.team5557.paths.pathfind.Node;
import org.team5557.paths.pathfind.Obstacle;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.state.goal.ObjectiveTracker;
import org.team5557.state.goal.ObjectiveTracker.Direction;
import org.team5557.subsystems.elevator.Elevator;
import org.team5557.subsystems.elevator.commands.ElevatorManual;
import org.team5557.subsystems.elevator.commands.HomeElevator;
import org.team5557.subsystems.elevator.commands.SetElevatorHeight;
import org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants;
import org.team5557.subsystems.intake.Intake;
import org.team5557.subsystems.intake.commands.IntakeShiver;
import org.team5557.subsystems.intake.commands.PassThrough;
import org.team5557.subsystems.intake.commands.SetIntakeState;
import org.team5557.subsystems.intake.util.IntakeState;
import org.team5557.subsystems.manipulator.Manipulator;
import org.team5557.subsystems.manipulator.commands.SetManipulatorState;
import org.team5557.subsystems.manipulator.util.ManipulatorState;
import org.team5557.subsystems.pneumatics.Pneumatics;
import org.team5557.subsystems.shoulder.Shoulder;
import org.team5557.subsystems.shoulder.commands.SetShoulderAngle;
import org.team5557.subsystems.shoulder.commands.ShoulderManual;
import org.team5557.subsystems.shoulder.util.ShoulderSubsystemConstants;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.Swerve.DriveMode;
import org.team5557.subsystems.swerve.commands.AimDrive;
import org.team5557.subsystems.swerve.commands.AutoBalance;
import org.team5557.subsystems.swerve.commands.CoPilot;
import org.team5557.subsystems.swerve.commands.TeleopDrive;
import org.team5557.subsystems.swerve.util.RawControllers;
import org.team5557.subsystems.swerve.util.SwerveSubsystemConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Subsystems
  public static final Pneumatics pneumatics = new Pneumatics();
  public static final Swerve swerve = new Swerve();
  public static final Elevator elevator = new Elevator(ElevatorSubsystemConstants.kElevatorConstants);
  public static final Shoulder shoulder = new Shoulder(ShoulderSubsystemConstants.kShoulderConstants);
  public static final Manipulator manipulator = new Manipulator();
  public static final Intake intake = new Intake();

  // Controller
  public static final XboxController primary_controller = new XboxController(Constants.ports.primary_controller);
  public static final XboxController danny_controller = new XboxController(Constants.ports.danny_controller);

  // Dashboard inputs / Util
  private final AutonomousChooser autonomous_chooser = new AutonomousChooser(new AutonomousTrajectories());

  private static final List<Obstacle> obstacles = FieldConstants.obstacles;
  public static final Pathweaver path_weaver = new Pathweaver(0, obstacles);

  public static final RawControllers raw_controllers = new RawControllers();
  public static final RobotStateSupervisor state_supervisor = new RobotStateSupervisor();
  public static final ObjectiveTracker objective_tracker = new ObjectiveTracker();

  //public static final PneumaticHub ph = new PneumaticHub(0);
  // private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  //Commands
  public static final Command homeElevatorCommand = new HomeElevator();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Constants.tuning_mode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    CommandScheduler.getInstance().registerSubsystem(state_supervisor);
    CommandScheduler.getInstance().registerSubsystem(pneumatics);

    configureButtonBindings();

    configurePathWeaver();
  }

  private void configureButtonBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    ///////////DRIVING\\\\\\\\\\\\\
    swerve.setDefaultCommand(
      new TeleopDrive(this::getForwardInput, this::getStrafeInput, this::getRotationInput)
    );

    new Trigger(primary_controller::getStartButton).onTrue(
      new InstantCommand(() -> swerve.setPose(new Pose2d()), swerve)
    );

    ////////////SCORING\\\\\\\\\\\\\\
    Command copilot = new CoPilot(this::getForwardInput, this::getStrafeInput);
    new Trigger(primary_controller::getRightBumper).whileTrue(
      copilot
    );

    /*
    Command setScorePosition = new SetSuperstructureSetpoint(objective_tracker::getDesiredSuperstructureState);
    new Trigger(() -> primary_controller.getRightTriggerAxis() >= 0.2)
      .whileTrue(setScorePosition)
      .onFalse(new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING.getState()));
      */

    //new Trigger(() -> copilot.atPose() && setScorePosition.atSetpoint()).onTrue(new SignalEjectGamepiece());
    

    ///////////INTAKING\\\\\\\\\\\\\\\\
    Command passThroughCube = new PassThrough();
    Command intakeCube = new SetIntakeState(IntakeState.IntakeStates.INTAKING_CUBE.getIntakeState()).withName("Intaking Cube");
    Command ejectCube = new SetIntakeState(IntakeState.IntakeStates.EJECT_CUBE.getIntakeState()).withName("Ejecting Cube");
    Command stopIntake = new SetIntakeState(IntakeState.IntakeStates.DO_NOTHING.getIntakeState()).withName("Stop Intake");
    Command intakeShiver = new IntakeShiver().withName("Controller Shiver");

    new Trigger(() -> primary_controller.getLeftTriggerAxis() > 0.5).whileTrue(
      passThroughCube.alongWith(
        intakeShiver
      )
    ).onFalse(stopIntake);

    new Trigger(() -> primary_controller.getRightTriggerAxis() > 0.5).whileTrue(
      ejectCube
    ).onFalse(stopIntake);

    ///////////MANIPULATING\\\\\\\\\\
    Command scoopCube = new SetManipulatorState(ManipulatorState.ManipulatorStates.INTAKING_CUBE.getManipulatorState());
    Command scoopCone = new SetManipulatorState(ManipulatorState.ManipulatorStates.INTAKING_CONE.getManipulatorState());
    Command stopManipulator = new SetManipulatorState(ManipulatorState.ManipulatorStates.DO_NOTHING.getManipulatorState());
    Command setShoulderConeIntake = new SetShoulderAngle(193.5);
    Command setShoulderCubeIntake = new SetShoulderAngle(320.0);

    new Trigger(() -> primary_controller.getRightTriggerAxis() > 0.5).whileTrue(
      scoopCone
    ).onFalse(stopManipulator);

    new Trigger(() -> primary_controller.getLeftTriggerAxis() > 0.5).whileTrue(
      scoopCube
    ).onFalse(stopManipulator);


    new Trigger(() -> danny_controller.getLeftTriggerAxis() > 0.5).onTrue(
      setShoulderCubeIntake
    );


    new Trigger(() -> danny_controller.getAButton()).whileTrue(
      setShoulderCubeIntake
    );

    /////////CHARGE STATION\\\\\\\\\\\
    new Trigger(primary_controller::getXButton).whileTrue(new AutoBalance());

    new Trigger(() -> danny_controller.getLeftTriggerAxis() > 0.5 && danny_controller.getRightTriggerAxis() > 0.5).whileTrue(
      new RunCommand(() -> swerve.setDriveMode(DriveMode.X_OUT), swerve)
    );

    new Trigger(() -> primary_controller.getPOV() == 0).whileTrue(
      new AimDrive(this::getForwardInput, this::getStrafeInput, 0.0)
    );

    new Trigger(() -> primary_controller.getPOV() == 90).whileTrue(
      new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI * 0.5)
    );

    new Trigger(() -> primary_controller.getPOV() == 180).whileTrue(
      new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI)
    );

    new Trigger(() -> primary_controller.getPOV() == 270).whileTrue(
      new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI * 1.5)
    );

    ////////////\\\\\\\\\\\\
    /////////DANNY\\\\\\\\\\
    ////////////\\\\\\\\\\\\



    //Manual elevator control
    Command manualElevatorControl = new ElevatorManual(() -> getElevatorJogger());
    new Trigger(() -> danny_controller.getLeftBumper()).whileTrue(
      manualElevatorControl
    );

    Command manualShoulderControl = new ShoulderManual(() -> getShoulderJogger());
    new Trigger(()-> danny_controller.getRightBumper()).whileTrue(
      manualShoulderControl
    );

    Command setElevatorCommand = new SetElevatorHeight(12.0);
    new Trigger(() -> danny_controller.getBButton() && false).whileTrue(
      setElevatorCommand
    );



    //Trigger Homing Command
    new Trigger(() -> danny_controller.getRightBumper() && danny_controller.getLeftBumper() && danny_controller.getAButton()).whileTrue(
      homeElevatorCommand
    );

    //Adjust Scoring objectives
    new Trigger(() -> danny_controller.getPOV() == 0).whileTrue(
      objective_tracker.shiftNodeCommand(Direction.DOWN)
    );
    new Trigger(() -> danny_controller.getPOV() == 90).whileTrue(
      objective_tracker.shiftNodeCommand(Direction.RIGHT)
    );
    new Trigger(() -> danny_controller.getPOV() == 180).whileTrue(
      objective_tracker.shiftNodeCommand(Direction.UP)
    );
    new Trigger(() -> danny_controller.getPOV() == 270).whileTrue(
      objective_tracker.shiftNodeCommand(Direction.LEFT)
    );
  }

  private void configurePathWeaver() {
    path_weaver.addNode(new Node(new Pose2d()));

    path_weaver.addNode(new Node(1,0));
    path_weaver.addNode(new Node(2.92-0.42,4.75));
    path_weaver.addNode(new Node(2.92-0.42,1.51-0.42));
    path_weaver.addNode(new Node(6,4.75));
    path_weaver.addNode(new Node(6,1.51-0.42));

    path_weaver.generateNodeEdges();
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
    return -square(deadband(primary_controller.getLeftY(), 0.1))
        * SwerveSubsystemConstants.MAX_VELOCITY_METERS_PER_SECOND;
  }

  public double getElevatorJogger() {
    return -square(deadband(danny_controller.getLeftY(), 0.1));
  }

  public double getShoulderJogger() {
    return -square(deadband(danny_controller.getLeftY(), 0.1));
  }

  public double getStrafeInput() {
    return -square(deadband(primary_controller.getLeftX(), 0.1))
        * SwerveSubsystemConstants.MAX_VELOCITY_METERS_PER_SECOND;
  }

  public double getRotationInput() {
    return -square(deadband(primary_controller.getRightX(), 0.1))
        * SwerveSubsystemConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  /**
   * @return Rotation2d of the driver's right stick angle with up being 0 and left
   *         being PI/2
   */
  public Rotation2d getRightStickAngle() {
    return new Rotation2d(deadband(-primary_controller.getRightY(), 0.4), deadband(-primary_controller.getRightX(), 0.4));
  }
}
