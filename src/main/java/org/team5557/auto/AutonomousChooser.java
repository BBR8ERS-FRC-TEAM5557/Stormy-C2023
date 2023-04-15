package org.team5557.auto;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.RobotContainer;
import org.team5557.commands.superstructure.SetSuperstructureSetpoint;
import org.team5557.commands.superstructure.SuperstructureAuto;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.subsystems.intake.commands.IntakeAuto;
import org.team5557.subsystems.manipulator.commands.ManipulatorAuto;
import org.team5557.subsystems.shoulder.commands.SetShoulderAngle;
import org.team5557.subsystems.swerve.Swerve.DriveMode;
import org.team5557.subsystems.swerve.commands.AutoBalance;
import org.team5557.subsystems.swerve.commands.FeedForwardCharacterization;
import org.team5557.subsystems.swerve.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final LoggedDashboardChooser<AutonomousMode> autonomousModeChooser = new LoggedDashboardChooser<>(Constants.shuffleboard.driver_readout_key);

    private final AutoBalance engage = new AutoBalance();

    public static HashMap<String, Command> eventMap = new HashMap<>();
    static {
        //Intake
        eventMap.put("none", new InstantCommand());
        eventMap.put("startIntaking", IntakeAuto.startIntaking());
        eventMap.put("stopIntaking", IntakeAuto.stopIntaking());
        eventMap.put("spitCube", IntakeAuto.spitCube());
        eventMap.put("startSpitting", IntakeAuto.startSpitting());


        eventMap.put("ejectCube", ManipulatorAuto.ejectCube());
        eventMap.put("fireCube", ManipulatorAuto.fireCube());
        eventMap.put("startSuckingCube", ManipulatorAuto.startSuckingCube());
        eventMap.put("startSuckingCone", ManipulatorAuto.startSuckingCone());
        eventMap.put("stopManipulator", ManipulatorAuto.stopManipulator());
        eventMap.put("holdCube", ManipulatorAuto.holdCube());
        eventMap.put("holdCone", ManipulatorAuto.holdCone());

        //Superstructure
        eventMap.put("holdItem", new SetSuperstructureSetpoint(SuperstructureState.Preset.HOLDING.getState()));
        eventMap.put("prepCubeSuck", new SetSuperstructureSetpoint(SuperstructureState.Preset.INTAKING_CUBE.getState()));
        eventMap.put("prepConeSuck", new SetSuperstructureSetpoint(SuperstructureState.Preset.INTAKING_CONE.getState()));
    }
    
    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.addDefaultOption("DO NOTHING", AutonomousMode.DO_NOTHING);
        autonomousModeChooser.addDefaultOption("Spit-Charge-Center", AutonomousMode.SPIT_CHARGE_CENTER);

        autonomousModeChooser.addDefaultOption("2+1-Charge-N-(eie)", AutonomousMode.TWO_PLUS_ONE_CHARGE_N_EIE);
        autonomousModeChooser.addDefaultOption("3-PARK-N-(eii)", AutonomousMode.THREE_PARK_N_EII);
        autonomousModeChooser.addDefaultOption("2-Park-N-(ee)", AutonomousMode.TWO_PARK_N_EE);
        autonomousModeChooser.addDefaultOption("2-Charge-N-(ee)", AutonomousMode.TWO_CHARGE_N_EE);
    }

    public LoggedDashboardChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    private void follow(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        FollowPathWithEvents auto = new FollowPathWithEvents(
            getPathFollowingCommand(trajectory),
            trajectory.getMarkers(),
            eventMap
        );
        command.addCommands(auto);
    }

    public void resetRobotPose(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        Pose2d start = trajectory.getInitialHolonomicPose();
        command.addCommands(new InstantCommand(() -> RobotContainer.swerve.setPose(start)));
    }

    private Command getPathFollowingCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
            trajectory, 
            () -> RobotContainer.swerve.getPose(), // Pose supplier
            RobotContainer.raw_controllers.xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            RobotContainer.raw_controllers.yController, // Y controller (usually the same values as X controller)
            RobotContainer.raw_controllers.rotationController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            chassisSpeed -> RobotContainer.swerve.drive(chassisSpeed), // Module states consumer
            false, //flip path based on alliance
            RobotContainer.swerve // Requires this drive subsystem
        );
    }


    public Command getSpitChargeCenter() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getSpit_Charge_Center());
        command.addCommands(IntakeAuto.spitCube());
        follow(command, trajectories.getSpit_Charge_Center());
        command.addCommands(new AutoBalance().withTimeout(10.0));
        command.addCommands(new RunCommand(() -> RobotContainer.swerve.drive(new ChassisSpeeds(), DriveMode.X_OUT, false, Constants.superstructure.center_of_rotation), RobotContainer.swerve));

        return command;
    }

    public Command get21ChargeNeie() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get21ChargeNeie());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get21ChargeNeie());
        command.addCommands(ManipulatorAuto.fireCube());
        command.addCommands(new AutoBalance());

        return command;
    }

    public Command get3ParkNeii() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get3ParkNeii());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get3ParkNeii());

        return command;
    }

    public Command get2parkNee() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2parkNee());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get2parkNee());

        return command;
    }

    public Command get2parkNeeADDON() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2parkNee());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get2parkNee());

        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get2parkNeeADDON());
        command.addCommands(new AutoBalance());

        return command;
    }

    public Command getFFCharacterization() {
        return new FeedForwardCharacterization(RobotContainer.swerve, true, new FeedForwardCharacterizationData("drive"), RobotContainer.swerve::runCharacterizationVolts, RobotContainer.swerve::getCharacterizationVelocity);
    }

    public Command getCommand() {
        switch (autonomousModeChooser.get()) {
            case SPIT_CHARGE_CENTER:
                return getSpitChargeCenter();
            
            case TWO_PLUS_ONE_CHARGE_N_EIE:
                return get21ChargeNeie();
            case THREE_PARK_N_EII:
                return get3ParkNeii();
            case TWO_PARK_N_EE:
                return get2parkNee();
            case TWO_CHARGE_N_EE:
                return get2parkNeeADDON();
            
            case FF_CHARACTERIZATION :
                return getFFCharacterization();
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        SPIT_CHARGE_CENTER,

        TWO_PLUS_ONE_CHARGE_N_EIE,
        THREE_PARK_N_EII,
        TWO_PARK_N_EE,
        TWO_CHARGE_N_EE,

        FF_CHARACTERIZATION,
        DO_NOTHING
    }
}