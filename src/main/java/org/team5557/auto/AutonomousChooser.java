package org.team5557.auto;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.RobotContainer;
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
        eventMap.put("startIntaking", IntakeAuto.startIntaking());
        eventMap.put("stopIntaking", IntakeAuto.stopIntaking());
        eventMap.put("spitCube", IntakeAuto.spitCube());
        eventMap.put("startPassthroughCube", IntakeAuto.passThroughCube());

        eventMap.put("ejectCube", ManipulatorAuto.ejectCube());
        eventMap.put("prepLowCube", Commands.sequence(new SetShoulderAngle(SuperstructureState.Preset.LOW_CUBE.getState().shoulder)));
        eventMap.put("prepIntakeCube", Commands.sequence(new SetShoulderAngle(SuperstructureState.Preset.INTAKING_CUBE.getState().shoulder)));

        eventMap.put("startSuckingCube", ManipulatorAuto.startSuckingCube());
        eventMap.put("stopManipulator", ManipulatorAuto.stopManipulator());
    }
    
    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.addDefaultOption("DO NOTHING", AutonomousMode.DO_NOTHING);
        autonomousModeChooser.addDefaultOption("Spit-Charge-Center", AutonomousMode.SPIT_CHARGE_CENTER);

        autonomousModeChooser.addOption("3-Charge-NoBump", AutonomousMode.THREE_CHARGE_NOBUMP);
        autonomousModeChooser.addOption("3-Park-NoBump", AutonomousMode.THREE_PARK_NOBUMP);
        autonomousModeChooser.addOption("2+1-Charge-NoBump", AutonomousMode.TWOPLUSONE_CHARGE_NOBUMP);

        autonomousModeChooser.addOption("I-3-Charge-NoBump", AutonomousMode.I_THREE_CHARGE_NOBUMP);
        autonomousModeChooser.addOption("I-3-Park-NoBump", AutonomousMode.I_THREE_PARK_NOBUMP);
        autonomousModeChooser.addOption("I-2+1-Charge-NoBump", AutonomousMode.I_TWOPLUSONE_CHARGE_NOBUMP);

        autonomousModeChooser.addOption("RED - Push and Charge", AutonomousMode.R_PUSH_AND_CHARGE);
        autonomousModeChooser.addOption("BLUE - Push and Charge", AutonomousMode.B_PUSH_AND_CHARGE);
        autonomousModeChooser.addOption("FF Characterization", AutonomousMode.FF_CHARACTERIZATION);
    }

    public LoggedDashboardChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getSpitChargeCenter() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getRPushAndCharge());//trajectories.getSpit_Charge_Center());
        command.addCommands(IntakeAuto.spitCube());
        follow(command, trajectories.getRPushAndCharge());//getSpit_Charge_Center());
        command.addCommands(engage.withTimeout(10.0));
        command.addCommands(new RunCommand(() -> RobotContainer.swerve.drive(new ChassisSpeeds(), DriveMode.X_OUT, false, Constants.superstructure.center_of_rotation), RobotContainer.swerve));

        return command;
    }
    //Regular
    public Command get3ChargeNoBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get3_Charge_NoBump());
        follow(command, trajectories.get3_Charge_NoBump());
        command.addCommands(engage);

        return command;
    }

    public Command get3ParkNoBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get3_Park_NoBump());
        follow(command, trajectories.get3_Park_NoBump());

        return command;
    }
    public Command get2plus1ChargeNoBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2plus1_Charge_NoBump());
        follow(command, trajectories.get2plus1_Charge_NoBump());
        command.addCommands(engage);

        return command;
    }
    public Command get1plus1ChargeBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get1plus1_Charge_Bump());
        follow(command, trajectories.get1plus1_Charge_Bump());
        command.addCommands(engage);

        return command;
    }

    //INTAKE ONLY
    public Command getI3ChargeNoBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getI_3_Charge_NoBump());
        follow(command, trajectories.getI_3_Charge_NoBump());
        command.addCommands(engage);

        return command;
    }

    public Command getI3ParkNoBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getI_3_Park_NoBump());
        follow(command, trajectories.getI_3_Park_NoBump());

        return command;
    }
    public Command getI2plus1ChargeNoBump() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getI_2plus1_Charge_NoBump());
        follow(command, trajectories.getI_2plus1_Charge_NoBump());
        command.addCommands(engage);

        return command;
    }

    //RANDOM STUFF
    public Command getBPushAndCharge() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getBPushAndCharge());
        follow(command, trajectories.getBPushAndCharge());
        command.addCommands(engage);

        return command;
    }

    public Command getRPushAndCharge() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getRPushAndCharge());
        follow(command, trajectories.getRPushAndCharge());
        command.addCommands(engage);

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
        Pose2d start = FieldConstants.allianceFlip(trajectory.getInitialPose());

        command.addCommands(new InstantCommand(() -> RobotContainer.swerve.setPose(start)));
    }

    private Command getPathFollowingCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
            trajectory, 
            () -> FieldConstants.allianceFlip(RobotContainer.swerve.getPose()), // Pose supplier
            RobotContainer.raw_controllers.xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            RobotContainer.raw_controllers.yController, // Y controller (usually the same values as X controller)
            RobotContainer.raw_controllers.rotationController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            chassisSpeed -> RobotContainer.swerve.drive(chassisSpeed), // Module states consumer
            true, //flip path based on alliance
            RobotContainer.swerve // Requires this drive subsystem
        );
    }

    public Command getCommand() {
        switch (autonomousModeChooser.get()) {
            case SPIT_CHARGE_CENTER:
                return getSpitChargeCenter();
            case THREE_CHARGE_NOBUMP:
                return get3ChargeNoBump();
            case THREE_PARK_NOBUMP:
                return get3ParkNoBump();
            case TWOPLUSONE_CHARGE_NOBUMP:
                return get2plus1ChargeNoBump();
            case ONEPLUSONE_CHARGE_BUMP:
                return get1plus1ChargeBump();


            case I_THREE_CHARGE_NOBUMP:
                return getI3ChargeNoBump();
            case I_THREE_PARK_NOBUMP:
                return getI3ParkNoBump();
            case I_TWOPLUSONE_CHARGE_NOBUMP:
                return getI2plus1ChargeNoBump();


            case R_PUSH_AND_CHARGE :
                return getRPushAndCharge();
            case B_PUSH_AND_CHARGE :
                return getBPushAndCharge();
            case FF_CHARACTERIZATION :
                return getFFCharacterization();
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        SPIT_CHARGE_CENTER,

        THREE_CHARGE_NOBUMP,
        THREE_PARK_NOBUMP,
        TWOPLUSONE_CHARGE_NOBUMP,
        ONEPLUSONE_CHARGE_BUMP,

        I_THREE_CHARGE_NOBUMP,
        I_THREE_PARK_NOBUMP,
        I_TWOPLUSONE_CHARGE_NOBUMP,

        R_PUSH_AND_CHARGE,
        B_PUSH_AND_CHARGE,

        FF_CHARACTERIZATION,
        DO_NOTHING
    }
}