package org.team5557.subsystems.swerve.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.state.RobotStateSupervisor.LocalizationStatus;
import org.team5557.state.RobotStateSupervisor.RobotState;
import org.team5557.state.goal.ObjectiveTracker;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.Swerve.DriveMode;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CoPilot extends CommandBase {
    private final PPHolonomicDriveController follower;
    private final PIDController alignController;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final Swerve swerve;
    private final RobotStateSupervisor state;

    private final PeriodicIO m_periodicIO;
    //private final LEDs leds;

    private double regenerationTimestamp;
    private double currentTimestamp;


    public CoPilot(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_periodicIO = new PeriodicIO();

        this.swerve = RobotContainer.swerve;
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

        int column = state.getColumn();



        if(m_periodicIO.active_trajectory == null) {
            Translation2d goalPoint;
            Translation2d alignmentPoint;
            Rotation2d goalRotation;
            double goalX;
            double goalY;

            boolean isBlue;
            Translation2d robotToAlignment;
            Translation2d alignmentToTarget;


            isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);

            goalX = FieldConstants.Grids.outerX + Constants.copilot.bumper_offset + Constants.copilot.chassis_offset;
            goalY = isBlue ? FieldConstants.Grids.lowTranslations[8 - column].getY() : FieldConstants.Grids.lowTranslations[column].getY();

            alignmentPoint = 
                new Translation2d(
                    goalX + Constants.copilot.alignment_offset, //drivebase offset thing
                    goalY
                );

            goalPoint = 
                new Translation2d(
                    goalX, //drivebase offset thing
                    goalY
                );

            goalPoint = FieldConstants.allianceFlip(goalPoint);
            goalRotation = isBlue ? new Rotation2d() : Rotation2d.fromDegrees(180);

            robotToAlignment = alignmentPoint.minus(m_periodicIO.state.estimatedPose.getTranslation());
            alignmentToTarget = goalPoint.minus(alignmentPoint);

            Translation2d initialVelocity = new Translation2d(swerve.getMeasuredVelocity().vxMetersPerSecond, swerve.getMeasuredVelocity().vyMetersPerSecond);
            Rotation2d initialVelocityHeading = initialVelocity.getAngle();
            double initialSpeed = initialVelocity.getNorm();

            if(initialSpeed < 0.5) {
                initialVelocityHeading = robotToAlignment.getAngle();
                initialSpeed = 0.0;
            }
            m_periodicIO.active_trajectory = 
            PathPlanner.generatePath(
                Constants.pathplanner.slow_constraints, 
                new PathPoint(
                    m_periodicIO.state.estimatedPose.getTranslation(),
                    initialVelocityHeading,
                    swerve.getPose().getRotation(),
                    initialSpeed
                ),
                new PathPoint(
                    alignmentPoint,
                    robotToAlignment.getAngle(),
                    goalRotation
                ),
                new PathPoint(
                    goalPoint,
                    alignmentToTarget.getAngle(),
                    goalRotation
                )
            );
            this.regenerationTimestamp = this.currentTimestamp;
        }

        if(m_periodicIO.active_trajectory != null && state.getVisionActivated()) {
            m_periodicIO.desired_state = (PathPlannerState) m_periodicIO.active_trajectory.sample(currentTimestamp - regenerationTimestamp);
            Pose2d currentPose = m_periodicIO.state.estimatedPose;
            m_periodicIO.target_chassis_speeds = this.follower.calculate(currentPose, m_periodicIO.desired_state);

            boolean isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
            Rotation2d goalRotation = isBlue ? new Rotation2d() : Rotation2d.fromDegrees(180);
            
            m_periodicIO.target_chassis_speeds.omegaRadiansPerSecond = RobotContainer.raw_controllers.calculateTheta(goalRotation.getRadians());
            
            //potentially use if the rotation on the path sucks
            //m_periodicIO.target_chassis_speeds.omegaRadiansPerSecond = RobotContainer.raw_controllers.calculateAlign(goalRotation.getRadians());
        } else {
            boolean isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);
            Rotation2d goalRotation = isBlue ? new Rotation2d() : Rotation2d.fromDegrees(180);

            double multiplier = 1.0;
            if(!isBlue) {
                multiplier = -1.0;
            }


            m_periodicIO.target_chassis_speeds = 
                new ChassisSpeeds(
                    translationXSupplier.getAsDouble() * multiplier, 
                    translationYSupplier.getAsDouble() * multiplier, 
                    RobotContainer.raw_controllers.calculateTheta(goalRotation.getRadians()));
            //System.out.println(m_periodicIO.target_chassis_speeds);
        }

        //Logger.getInstance().recordOutput("CoPilot/Active Trajectory", m_periodicIO.active_trajectory);
        Logger.getInstance().recordOutput("CoPilot/At Goal", atGoal());
        Logger.getInstance().recordOutput("CoPilot/TargetSpeeds", m_periodicIO.target_chassis_speeds.omegaRadiansPerSecond);
        Logger.getInstance().recordOutput("CoPilot/Goal Pose", m_periodicIO.active_trajectory.getEndState().poseMeters);

        //Logger.getInstance().recordOutput("CoPilot/Next Pose", m_periodicIO.desired_state.poseMeters);
        //Logger.getInstance().recordOutput("CoPilot/Next Velocity", m_periodicIO.desired_state.velocityMetersPerSecond);
        //Logger.getInstance().recordOutput("CoPilot/Next Acceleration", m_periodicIO.desired_state.accelerationMetersPerSecondSq);
    }

    public synchronized void writeOutputs() {
        swerve.drive(
            m_periodicIO.target_chassis_speeds,
            DriveMode.OPEN_LOOP,
            true, //??
            Constants.superstructure.center_of_rotation
        );
    }

    public synchronized boolean atGoal() {
        return follower.atReference();
    }

    public static class PeriodicIO {
        // INPUTS
        public LocalizationStatus localizationStatus;
        public RobotState state;
        public PathPlannerState desired_state;
  
        // OUTPUTS
        public PathPlannerTrajectory active_trajectory;
        public DriveMode driveMode;
        public ChassisSpeeds target_chassis_speeds;
    }

}
