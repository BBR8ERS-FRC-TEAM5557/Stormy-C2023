package org.team5557.state;

import java.util.ArrayList;
import java.util.Map;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.state.vision.VisionManager;
import org.team5557.state.vision.VisionTarget;
import org.team5557.subsystems.swerve.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.library.team2910.util.MovingAverage;

public class RobotStateSupervisor extends SubsystemBase {

    private RobotState robot_state = new RobotState();
    private final Swerve swerve = RobotContainer.swerve;
    private final VisionManager vision_manager = new VisionManager();
    //private final GoalPlanner goal_planner = new GoalPlanner();
    private final SwerveDrivePoseEstimator pose_estimator = swerve.getEstimator();

    private final GenericEntry skidDeaccumulationSensitivity;

    public RobotStateSupervisor() {
        robot_state.visionToEstimatorDisagreement = new MovingAverage(30);

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.supervisor_readout_key);

        tab.add("Localization", robot_state.localizationStatus.toString());
        tab.add("Skid Accumulator", robot_state.skidAccumulator.getNorm());
        tab.add("Vision - Estimator Disagreement", robot_state.visionToEstimatorDisagreement.get());

        skidDeaccumulationSensitivity = tab.add("Skid Deaccumulation Sensitivity", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.1, "max", 50.0, "Block increment", 10.0))
            .getEntry();
    }


    @Override
    public void periodic() {
        //Update general info
        robot_state.estimatedPose = pose_estimator.getEstimatedPosition();

        //Update Vision Manager
        this.vision_manager.update();
        //recieve list of vision measurements from VM
        robot_state.visionTargets = vision_manager.getTargets();
        //add them to the pose estimator
        for (VisionTarget target : robot_state.visionTargets) {
            //add the measurement to the estimator
            pose_estimator.addVisionMeasurement(target.measuredPose, target.timestamp);

            //add the estimator vs. vision measurement to the moving average to determine convergence
            robot_state.visionToEstimatorDisagreement.add(target.measuredPose.minus(robot_state.estimatedPose).getTranslation().getNorm());

            //find the delta caused by adding the vision measurement and subtract it from the skid error radius
            Translation2d delta = robot_state.estimatedPose.getTranslation().minus(pose_estimator.getEstimatedPosition().getTranslation());
            this.subtractSkidMeasurement(delta.times(skidDeaccumulationSensitivity.getDouble(2.0)));

            //update general info
            robot_state.estimatedPose = pose_estimator.getEstimatedPosition();
        }

        if (robot_state.skidAccumulator.getNorm() < Constants.supervisor.maxSkidErrorMeters) {
            robot_state.localizationStatus = LocalizationStatus.LOCALIZING;
            if(robot_state.visionToEstimatorDisagreement.get() < Constants.supervisor.visionToEstimatorConvergenceThreshold && robot_state.visionToEstimatorDisagreement.get() >= 0 ) {
                clearSkidAccumulator();
                robot_state.localizationStatus = LocalizationStatus.LOCALIZED;
            }
        } else {
            clearVisionEstimatorDisagreement();
            robot_state.localizationStatus = LocalizationStatus.DELOCALIZED;
        }
    }

    /*
     * Adds to the accumulated skid error that is being stored as a chassis speed
     * @param measuredVelocity - the measurement of the chassis speed straight from the encoders
     * @param filteredVelocity - the measurement of the chassis speed after removing any detected instances of skid
     */
    public void addSkidMeasurement(ChassisSpeeds measuredVelocity, ChassisSpeeds filteredVelocity) {
        robot_state.skidAccumulator = 
            new Translation2d(
                robot_state.skidAccumulator.getX() + Math.abs(measuredVelocity.vxMetersPerSecond - filteredVelocity.vxMetersPerSecond) * Constants.kloop_period,
                robot_state.skidAccumulator.getY() + Math.abs(measuredVelocity.vyMetersPerSecond - filteredVelocity.vyMetersPerSecond) * Constants.kloop_period
            );
    }

    private void subtractSkidMeasurement(Translation2d delta) {
        double x = robot_state.skidAccumulator.getX() - Math.abs(delta.getX());
        double y = robot_state.skidAccumulator.getY() - Math.abs(delta.getY());
        if(x < 0.0)
            x = 0.0;
        if(y < 0.0)
            y = 0.0;

        robot_state.skidAccumulator = new Translation2d(x,y);
    }

    private void clearSkidAccumulator() {
        robot_state.skidAccumulator = new Translation2d();
    }

    private void clearVisionEstimatorDisagreement() {
        robot_state.visionToEstimatorDisagreement.clear();
        robot_state.visionToEstimatorDisagreement.add(-0.0001);
    }

    public void setPose(Pose2d pose) {
        swerve.setPose(pose);
        robot_state.localizationStatus = LocalizationStatus.LOCALIZED;
        clearSkidAccumulator();
        clearVisionEstimatorDisagreement();
    }


    public RobotState getRobotState() {
        return robot_state;
    }

    public static class RobotState {
        //General Info
        Pose2d estimatedPose;
        LocalizationStatus localizationStatus;

        //Skid/Swerve Related
        Translation2d skidAccumulator;

        //Vision Related
        ArrayList<VisionTarget> visionTargets;
        MovingAverage visionToEstimatorDisagreement;
        
    }

    public enum LocalizationStatus {
        LOCALIZED,
        DELOCALIZED,
        LOCALIZING
    }
    
}
