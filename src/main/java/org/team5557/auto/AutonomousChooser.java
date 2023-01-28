package org.team5557.auto;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.commands.swerve.FeedForwardCharacterization;
import org.team5557.commands.swerve.FeedForwardCharacterization.FeedForwardCharacterizationData;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final LoggedDashboardChooser<AutonomousMode> autonomousModeChooser = new LoggedDashboardChooser<>(Constants.shuffleboard.driver_readout_key);

    public static HashMap<String, Command> eventMap = new HashMap<>();
    static {
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("intakeDown", new PrintCommand("THing Happened!!"));
    }
    
    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.addDefaultOption("Five Ball (Orange)", AutonomousMode.CIRCLE_AUTO);
        autonomousModeChooser.addOption("Test Auto", AutonomousMode.TEST_AUTO);
        autonomousModeChooser.addOption("FF Characterization", AutonomousMode.FF_CHARACTERIZATION);
    }

    public LoggedDashboardChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getTestAuto() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getTestAutoPartOne());
        follow(command, trajectories.getTestAutoPartOne());

        return command;
    }

    public Command getFFCharacterization() {
        return new FeedForwardCharacterization(RobotContainer.swerve, true, new FeedForwardCharacterizationData("drive"), RobotContainer.swerve::runCharacterizationVolts, RobotContainer.swerve::getCharacterizationVelocity);
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
        Pose2d start = trajectory.getInitialPose();
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
            RobotContainer.swerve // Requires this drive subsystem
        );
    }

    public Command getCommand() {
        switch (autonomousModeChooser.get()) {
            case TEST_AUTO :
                return getTestAuto();
            case FF_CHARACTERIZATION :
                return getFFCharacterization();
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        RANDOM_THING,
        TEST_AUTO,
        CIRCLE_AUTO,
        FF_CHARACTERIZATION
    }
}