// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import org.library.team6328.util.Alert;
import org.library.team6328.util.Alert.AlertType;
import org.team5557.auto.AutonomousChooser;
import org.team5557.auto.AutonomousTrajectories;
import org.team5557.commands.superstructure.SetSuperstructureSetpoint;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.state.goal.ObjectiveTracker;
import org.team5557.state.goal.ObjectiveTracker.Direction;
import org.team5557.state.vision.VisionManager;
import org.team5557.subsystems.elevator.Elevator;
import org.team5557.subsystems.elevator.commands.ElevatorManual;
import org.team5557.subsystems.elevator.commands.HomeElevator;
import org.team5557.subsystems.elevator.commands.SetElevatorHeight;
import org.team5557.subsystems.elevator.util.ElevatorSubsystemConstants;
import org.team5557.subsystems.intake.Intake;
import org.team5557.subsystems.intake.commands.IntakeAuto;
import org.team5557.subsystems.intake.commands.IntakeShiver;
import org.team5557.subsystems.intake.commands.PassThrough;
import org.team5557.subsystems.intake.commands.SetIntakeState;
import org.team5557.subsystems.intake.util.IntakeState;
import org.team5557.subsystems.leds.LEDs;
import org.team5557.subsystems.manipulator.Manipulator;
import org.team5557.subsystems.manipulator.commands.ManipulatorAuto;
import org.team5557.subsystems.manipulator.commands.ManipulatorShiver;
import org.team5557.subsystems.manipulator.commands.SetManipulatorState;
import org.team5557.subsystems.manipulator.commands.SmartEject;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Subsystems
  public static final Pneumatics pneumatics = new Pneumatics();
  public static final Swerve swerve = new Swerve();
  public static final Elevator elevator = new Elevator(ElevatorSubsystemConstants.kElevatorConstants);
  public static final Shoulder shoulder = new Shoulder(ShoulderSubsystemConstants.kShoulderConstants);
  public static final Manipulator manipulator = new Manipulator();
  public static final Intake intake = new Intake();
  public static final LEDs leds = new LEDs();

  // Controller
  public static final XboxController primary_controller = new XboxController(Constants.ports.primary_controller);
  public static final XboxController danny_controller = new XboxController(Constants.ports.danny_controller);

  // Dashboard inputs / Util
  private final AutonomousChooser autonomous_chooser = new AutonomousChooser(new AutonomousTrajectories());
  public static final RawControllers raw_controllers = new RawControllers();
  public static final RobotStateSupervisor state_supervisor = new RobotStateSupervisor();

  // Commands
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
  }

  private void configureButtonBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);

    /////////// DRIVING\\\\\\\\\\\\\
    swerve.setDefaultCommand(
        new TeleopDrive(this::getForwardInput, this::getStrafeInput, this::getRotationInput));

    new Trigger(primary_controller::getStartButton).onTrue(
        new InstantCommand(() -> swerve.setPoseTeleop(), swerve));

    Command copilot = new CoPilot(this::getForwardInput, this::getStrafeInput);
    new Trigger(primary_controller::getRightBumper).whileTrue(
        copilot);



    //MANUAL OPERATION FOR TESTING
    new Trigger(() -> danny_controller.getLeftBumper() && danny_controller.getRightBumper()).whileTrue(
        new ElevatorManual(() -> getElevatorJogger())
    );

    new Trigger(() -> danny_controller.getRightBumper() && danny_controller.getLeftBumper()).whileTrue(
        new ShoulderManual(() -> getShoulderJogger())
    );

    new Trigger(() -> danny_controller.getBButton() && false).whileTrue(
        new SetElevatorHeight(12.0)
    );

    new Trigger(
        () -> danny_controller.getRightBumper() && danny_controller.getLeftBumper() && danny_controller.getAButton())
        .whileTrue(
            homeElevatorCommand
    );


    //////////// \\\\\\\\\\\\
    //////// INTAKING\\\\\\\\
    //////////// \\\\\\\\\\\\

    //INTAKING CUBES WITH MANIPULATOR -> Josh hold left Trigger
    new Trigger(() -> primary_controller.getLeftTriggerAxis() > 0.5).whileTrue(
        new SetSuperstructureSetpoint(SuperstructureState.Preset.INTAKING_CUBE.getState(), this::getElevatorJogger)
            .alongWith(ManipulatorAuto.suckCubeStop())
    )
        .onFalse(new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING_CUBE.getState(), this::getElevatorJogger))
        .onFalse(ManipulatorAuto.holdCube());
    

    //INTAKING CUBES WITH INTAKE -> Josh hold right Trigger
    new Trigger(() -> primary_controller.getRightTriggerAxis() > 0.5).whileTrue(
        IntakeAuto.blockedCube()
            .alongWith(new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING_NADA.getState(), this::getElevatorJogger))
            .alongWith(ManipulatorAuto.stopManipulator())
    )
    .onFalse(IntakeAuto.stopIntaking());



    SuperstructureState followThruConeSetpoint = SuperstructureState.Preset.INTAKING_CONE_FOLLOW_THRU.getState();
    //INTAKING CONES WITH MANIPULATOR -> Josh hold left Bumper
    new Trigger(() -> primary_controller.getLeftBumper()).whileTrue(
        Commands.sequence(
            new SetSuperstructureSetpoint(SuperstructureState.Preset.INTAKING_CONE.getState())
                .alongWith(ManipulatorAuto.startSuckingCone())
                .until(manipulator::getConeDetected),

            new SetSuperstructureSetpoint(followThruConeSetpoint)
                .until(() -> state_supervisor.isAtDesiredState(followThruConeSetpoint)),

            new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING_CONE.getState())
                .alongWith(ManipulatorAuto.holdCone())
        )
    )
        .onFalse(new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING_CONE.getState()))
        .onFalse(ManipulatorAuto.holdCone());


                /*new SetSuperstructureSetpoint(SuperstructureState.Preset.INTAKING_CONE.getState())
            .alongWith(ManipulatorAuto.startSuckingCone())
            //UNCOMMENT THE LINES BELOW TO TEST THE FOLLOW THRU MOTION WHEN INTAKING A CONE
            //YOU MUST PRESS THE Y BUTTON ON PRIMARY CONTROLLER TO SIMULATE THE ROBOT DETECTING A CONE
            //IF YOU WIRE THE BEAM BREAK MAKE SURE TO PLUG IT INTO THE 3rd DIO PORT
            //IF IT DOESN'T TURN ON YOU PROBABLY PLUGGED IT IN BACKWARDS
            //REMEMBER YOU CAN CHANGE THE DETECTION DISTANCE BY LOOSENING AND TIGHTENING THE SCREW ON THE BACK
            .until(manipulator::getConeDetected)
            //.andThen(new SetSuperstructureSetpoint(followThruConeSetpoint))
            //.until(() -> state_supervisor.isAtDesiredState(followThruConeSetpoint))
            //.andThen(new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING_CONE.getState()).alongWith(ManipulatorAuto.holdCone()))*/



    //INTAKING SINGLE SUBSTATION -> Josh hold X
    new Trigger(() -> primary_controller.getXButton()).whileTrue(
        new SetSuperstructureSetpoint(SuperstructureState.Preset.INTAKING_CHUTE_CONE.getState(), this::getElevatorJogger)
            .alongWith(ManipulatorAuto.startSuckingCone())
            .alongWith(new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI * 1.5))
    )
        .onFalse(new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING_CONE.getState(), this::getElevatorJogger))
        .onFalse(ManipulatorAuto.holdCone());


    //////////// \\\\\\\\\\\\
    ////////SCORING\\\\\\\\\\
    //////////// \\\\\\\\\\\\
    new Trigger(() -> danny_controller.getRightTriggerAxis() > 0.5).whileTrue(
        //IntakeAuto.startBallasting().andThen(new SetSuperstructureSetpoint(() -> state_supervisor.getDesiredSuperstructureState(), this::getElevatorJogger))
        new SetSuperstructureSetpoint(() -> state_supervisor.getDesiredSuperstructureState(), this::getElevatorJogger)
    )
        .onFalse(new SetSuperstructureSetpoint(() -> state_supervisor.getDesiredHoldingSuperstructureState(), this::getElevatorJogger))
        .onFalse(IntakeAuto.stopIntaking());

    new Trigger(primary_controller::getAButton)
        .onTrue(new SmartEject())
        .onTrue(IntakeAuto.spitCube());


    ////////////////\\\\\\\\\\\\\\\\\\
    /////////CHARGE STATION\\\\\\\\\\\
    ////////////////\\\\\\\\\\\\\\\\\\
    new Trigger(primary_controller::getXButton).whileTrue(new AutoBalance());

    new Trigger(() -> danny_controller.getLeftTriggerAxis() > 0.5 && danny_controller.getRightTriggerAxis() > 0.5)
        .whileTrue(
            new RunCommand(() -> swerve.setDriveMode(DriveMode.X_OUT), swerve));

    new Trigger(() -> primary_controller.getPOV() == 0).whileTrue(
        new AimDrive(this::getForwardInput, this::getStrafeInput, 0.0));

    new Trigger(() -> primary_controller.getPOV() == 90).whileTrue(
        new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI * 0.5));

    new Trigger(() -> primary_controller.getPOV() == 180).whileTrue(
        new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI));

    new Trigger(() -> primary_controller.getPOV() == 270).whileTrue(
        new AimDrive(this::getForwardInput, this::getStrafeInput, Math.PI * 1.5));
    

    ////////////\\\\\\\\\\\\
    ////////OBJECTIVE\\\\\\\
    ////////////\\\\\\\\\\\\

    // Adjust Scoring objectives
    new Trigger(() -> danny_controller.getPOV() == 0).whileTrue(
        state_supervisor.shiftNodeCommand(Direction.UP));
    new Trigger(() -> danny_controller.getPOV() == 90).whileTrue(
        state_supervisor.shiftNodeCommand(Direction.RIGHT));
    new Trigger(() -> danny_controller.getPOV() == 180).whileTrue(
        state_supervisor.shiftNodeCommand(Direction.DOWN));
    new Trigger(() -> danny_controller.getPOV() == 270).whileTrue(
        state_supervisor.shiftNodeCommand(Direction.LEFT));


    new Trigger(() -> danny_controller.getXButton()).onTrue(
        new InstantCommand(() -> RobotContainer.state_supervisor.flipEnableVision()).ignoringDisable(true)
    );
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

  public static double square(double value) {
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
    return -square(deadband(danny_controller.getRightY(), 0.1));
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
    return new Rotation2d(deadband(-primary_controller.getRightY(), 0.4),
        deadband(-primary_controller.getRightX(), 0.4));
  }
}
