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
import org.team5557.subsystems.swerve.commands.NewAutoBalance;
import org.team5557.subsystems.swerve.commands.SwerveAuto;
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
        eventMap.put("startSuckingCube", ManipulatorAuto.suckCubeStop());
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

        autonomousModeChooser.addOption("3-PARK-N-(eii)", AutonomousMode.THREE_PARK_N_EII);
        autonomousModeChooser.addOption("2+1-Charge-N-(eie)", AutonomousMode.TWO_PLUS_ONE_CHARGE_N_EIE);


        autonomousModeChooser.addOption("2-Park-N-(ee)", AutonomousMode.TWO_PARK_N_EE);
        autonomousModeChooser.addOption("2-Charge-N-(ee)", AutonomousMode.TWO_CHARGE_N_EE);

        autonomousModeChooser.addOption("2-Park-N-(ei)", AutonomousMode.TWO_PARK_N_EI);
        autonomousModeChooser.addOption("2-Charge-N-(ei)", AutonomousMode.TWO_CHARGE_N_EI);

        autonomousModeChooser.addOption("2-Park-B-(ei)", AutonomousMode.TWO_PARK_B_EI);
        autonomousModeChooser.addOption("2-Charge-B-(ei)", AutonomousMode.TWO_CHARGE_B_EI);

        autonomousModeChooser.addOption("1+1-Charge-C-(ei)", AutonomousMode.ONE_PLUS_ONE_CHARGE_C_EI);
        autonomousModeChooser.addOption("1-Charge-C-(e)", AutonomousMode.SCORE_CHARGE_CENTER);
        autonomousModeChooser.addOption("Spit-Charge-Center", AutonomousMode.SPIT_CHARGE_CENTER);

        autonomousModeChooser.addDefaultOption("DO NOTHING", AutonomousMode.DO_NOTHING);
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


    ///////SPECIALS\\\\\\\
    //scores high cube --> intakes and scores --> intakes and scores
    public Command get3ParkNeii() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get3ParkNeii());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.get3ParkNeii());

        return command;
    }

    //scores high cube --> intakes and scores --> sucks and fires --> charges 
    public Command get21ChargeNeie() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get21ChargeNeie());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.get21ChargeNeie());
        command.addCommands(ManipulatorAuto.fireCube());
        command.addCommands(new AutoBalance());

        return command;
    }






    //////2 double high\\\\\\
    //scores high cone --> sucks cube --> scores high cube
    public Command get2parkNee() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2ParkNee());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get2ParkNee());
        command.addCommands(SuperstructureAuto.scoreHighCube());

        return command;
    }
    //scores high cone --> sucks cube --> scores high cube --> charges
    public Command get2chargeNee() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2ParkNee());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get2ParkNee());
        command.addCommands(SuperstructureAuto.scoreHighCube());

        follow(command, trajectories.get2ChargeAddNee());
        command.addCommands(new AutoBalance());

        return command;
    }




    //////2 no bump high-low\\\\\\
    //scores high cone --> intakes cube --> spits cube
    public Command get2parkNei() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2ParkNei());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.get2ParkNei());
        command.addCommands(IntakeAuto.spitCube());


        return command;
    }
    //scores high cone --> intakes cube --> spits cube --> charges
    public Command get2chargeNei() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2ParkNei());
        command.addCommands(SuperstructureAuto.scoreHighCone());
        follow(command, trajectories.get2ParkNei());
        command.addCommands(IntakeAuto.spitCube());
        follow(command, trajectories.get2ChargeAddNei());
        command.addCommands(new AutoBalance());

        return command;
    }





    //////2 bump high-low\\\\\\
    //scores high cone --> intakes cube --> spits cube
    public Command get2parkBei() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2ParkBei());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.get2ParkBei());

        return command;
    }
    //scores high cone --> intakes cube --> spits cube --> charges
    public Command get2chargeBei() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get2ParkBei());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.get2ParkBei());
        follow(command, trajectories.get2ChargeAddBei());
        command.addCommands(new AutoBalance());

        return command;
    }





    //////2 no bump high-low\\\\\\
    //scores high cone --> passes over station --> intakes cube --> charges
    public Command get11chargeCei() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.get11ChargeCeiPart1());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.get11ChargeCeiPart1());
        command.addCommands(SwerveAuto.driveTillStationFlip());
        command.addCommands(SwerveAuto.driveTillFlat());
        follow(command, trajectories.get11ChargeCeiPart3());
        command.addCommands(new AutoBalance());

        return command;
    }

    

    //spits cube --> charges
    public Command getSpitChargeCenter() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getSpit_Charge_Center());
        command.addCommands(IntakeAuto.spitCube());
        follow(command, trajectories.getSpit_Charge_Center());
        command.addCommands(new AutoBalance().withTimeout(10.0));
        command.addCommands(new RunCommand(() -> RobotContainer.swerve.drive(new ChassisSpeeds(), DriveMode.X_OUT, false, Constants.superstructure.center_of_rotation), RobotContainer.swerve));

        return command;
    }

    //spits cube --> charges
    public Command getScoreChargeCenter() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getScore_Charge_Center());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.getScore_Charge_Center());
        command.addCommands(new AutoBalance().withTimeout(10.0));
        command.addCommands(new RunCommand(() -> RobotContainer.swerve.drive(new ChassisSpeeds(), DriveMode.X_OUT, false, Constants.superstructure.center_of_rotation), RobotContainer.swerve));

        return command;
    }

    public Command getScoreChargeOverCenter() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectories.getScore_Charge_Center());
        command.addCommands(SuperstructureAuto.scoreHighCube());
        follow(command, trajectories.getScore_Charge_Center());

        command.addCommands(SwerveAuto.driveTillStationFlip());
        command.addCommands(SwerveAuto.driveTillFlat());

        follow(command, trajectories.getOneChargeCePart2());

        command.addCommands(new NewAutoBalance());

        return command;
    }







    public Command getFFCharacterization() {
        return new FeedForwardCharacterization(RobotContainer.swerve, true, new FeedForwardCharacterizationData("drive"), RobotContainer.swerve::runCharacterizationVolts, RobotContainer.swerve::getCharacterizationVelocity);
    }

    public Command getCommand() {
        switch (autonomousModeChooser.get()) {
            case THREE_PARK_N_EII:
                return get3ParkNeii();
            case TWO_PLUS_ONE_CHARGE_N_EIE:
                return get21ChargeNeie();


            case TWO_PARK_N_EE:
                return get2parkNee();
            case TWO_CHARGE_N_EE:
                return get2chargeNee();


            case TWO_PARK_N_EI:
                return get2parkNei();
            case TWO_CHARGE_N_EI:
                return get2chargeNei();


            case TWO_PARK_B_EI:
                return get2parkBei();
            case TWO_CHARGE_B_EI:
                return get2chargeBei();



            case ONE_PLUS_ONE_CHARGE_C_EI:
                return get11chargeCei();
            case SCORE_CHARGE_OVER_C_E:
                return getScoreChargeOverCenter();
            case SCORE_CHARGE_CENTER:
                return getScoreChargeCenter();
            case SPIT_CHARGE_CENTER:
                return getSpitChargeCenter();
            



            
            case FF_CHARACTERIZATION :
                return getFFCharacterization();
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        THREE_PARK_N_EII,
        TWO_PLUS_ONE_CHARGE_N_EIE,


        TWO_PARK_N_EE,
        TWO_CHARGE_N_EE,


        TWO_PARK_N_EI,
        TWO_CHARGE_N_EI,


        TWO_PARK_B_EI,
        TWO_CHARGE_B_EI,

        ONE_PLUS_ONE_CHARGE_C_EI,
        SCORE_CHARGE_OVER_C_E,
        SCORE_CHARGE_CENTER,
        SPIT_CHARGE_CENTER,


        FF_CHARACTERIZATION,
        DO_NOTHING
    }
}