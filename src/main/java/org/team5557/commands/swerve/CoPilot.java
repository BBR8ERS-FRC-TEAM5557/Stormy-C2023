package org.team5557.commands.swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.paths.Pathweaver;
import org.team5557.paths.pathfind.Node;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.state.RobotStateSupervisor.LocalizationStatus;
import org.team5557.state.RobotStateSupervisor.RobotState;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.Swerve.DriveMode;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CoPilot extends CommandBase {
    private final PPHolonomicDriveController follower;
    private final PIDController alignController;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final Swerve swerve;
    private final RobotStateSupervisor state;
    private Pathweaver path_weaver;
    private final PeriodicIO m_periodicIO;
    //private final LEDs leds;

    private double regenerationTimestamp;
    private double currentTimestamp;


    public CoPilot(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_periodicIO = new PeriodicIO();

        this.swerve = RobotContainer.swerve;
        this.path_weaver = RobotContainer.path_weaver;
        this.state = RobotContainer.state_supervisor;
        this.follower = RobotContainer.raw_controllers.follower;
        this.alignController = RobotContainer.raw_controllers.alignController;
        //this.leds = RobotContainer.leds;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.alignController.reset();
        m_periodicIO.active_trajectory = null;
    }

    @Override
    public void execute() {
        this.readInputs();
        this.writeOutputs();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    public synchronized void readInputs() {
        this.currentTimestamp = Timer.getFPGATimestamp();

        m_periodicIO.state = state.getRobotState();

        m_periodicIO.localizationStatus = m_periodicIO.state.localizationStatus;

        if(m_periodicIO.active_trajectory == null && m_periodicIO.localizationStatus == LocalizationStatus.LOCALIZED) {
            //Translation2d goal = new Translation2d();
            //Translation2d robot_to_target = goal.minus(swerve.getPose().getTranslation());

            m_periodicIO.active_trajectory = path_weaver.generatePath(new Node(new Pose2d())); /*PathPlanner.generatePath(
                Constants.pathplanner.hellaslow_constraints, 
                new PathPoint(
                    swerve.getPose().getTranslation(),
                    robot_to_target.getAngle(),
                    swerve.getPose().getRotation()
                ),
                new PathPoint(
                    goal,
                    robot_to_target.getAngle(),
                    new Rotation2d()
                )
            );*/
            this.regenerationTimestamp = this.currentTimestamp;
        }

        if(m_periodicIO.active_trajectory != null) {
            m_periodicIO.desired_state = (PathPlannerState) m_periodicIO.active_trajectory.sample(currentTimestamp - regenerationTimestamp);
            Pose2d currentPose = m_periodicIO.state.estimatedPose;
            m_periodicIO.target_chassis_speeds = this.follower.calculate(currentPose, m_periodicIO.desired_state);
        } else {
            m_periodicIO.target_chassis_speeds = new ChassisSpeeds(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(), RobotContainer.raw_controllers.calculateAlign(0));
        }

        Logger.getInstance().recordOutput("CoPilot/Active Trajectory", m_periodicIO.active_trajectory);
    }

    public synchronized void writeOutputs() {
        swerve.drive(
            m_periodicIO.target_chassis_speeds,
            DriveMode.CLOSED_LOOP,
            false, //??
            Constants.superstructure.center_of_rotation
        );
    }

    public static class PeriodicIO {
        // INPUTS
        public LocalizationStatus localizationStatus;
        public Translation2d robot_to_target;
        public RobotState state;
        public PathPlannerState desired_state;
  
        // OUTPUTS
        public PathPlannerTrajectory active_trajectory;
        public DriveMode driveMode;
        public ChassisSpeeds target_chassis_speeds;
    }

}
