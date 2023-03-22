package org.team5557.state;

import org.library.team2910.util.MovingAverage;
import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.planners.superstructure.util.SuperstructureState;
import org.team5557.state.goal.ObjectiveTracker;
import org.team5557.state.goal.ObjectiveTracker.Direction;
import org.team5557.state.goal.ObjectiveTracker.GamePiece;
import org.team5557.state.goal.ObjectiveTracker.NodeLevel;
import org.team5557.state.vision.VisionManager;
import org.team5557.subsystems.swerve.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateSupervisor extends SubsystemBase {

    private RobotState robot_state = new RobotState();
    private final Swerve swerve = RobotContainer.swerve;
    private final VisionManager vision_manager = new VisionManager();
    private final ObjectiveTracker objective_tracker = new ObjectiveTracker();

    //private final GoalPlanner goal_planner = new GoalPlanner();
    private final SwerveDrivePoseEstimator pose_estimator = swerve.getEstimator();

    public RobotStateSupervisor() {
        vision_manager.setDataInterface(swerve::addVisionMeasurement, this::addVisionCheck);

        robot_state.visionToEstimatorDisagreement = new MovingAverage(30);
        robot_state.localizationStatus = LocalizationStatus.LOCALIZED;
        robot_state.skidAccumulator = new Translation2d();

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.supervisor_readout_key);

        tab.add("Localization", robot_state.localizationStatus.toString());
        tab.add("Skid Accumulator", robot_state.skidAccumulator.getNorm());
        tab.add("Vision - Estimator Disagreement", robot_state.visionToEstimatorDisagreement.get());
    }


    @Override
    public void periodic() {
        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
        ///////////////////GENERAL\\\\\\\\\\\\\\\\\\\\
        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\

        //Update general info
        robot_state.estimatedPose = pose_estimator.getEstimatedPosition();


        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
        ////////////////////VISION\\\\\\\\\\\\\\\\\\\\
        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
        
        //Update Vision Manager
        this.vision_manager.update();

        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
        ///////////////SKID DETECTION\\\\\\\\\\\\\\\\\
        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\

        if (robot_state.skidAccumulator.getNorm() < Constants.supervisor.maxSkidErrorMeters) {
            robot_state.localizationStatus = LocalizationStatus.LOCALIZING;
            if(robot_state.visionToEstimatorDisagreement.get() < Constants.supervisor.visionToEstimatorConvergenceThreshold) {
                clearSkidAccumulator();
                robot_state.localizationStatus = LocalizationStatus.LOCALIZED;
            }
        } else {
            clearVisionEstimatorDisagreement();
            robot_state.localizationStatus = LocalizationStatus.DELOCALIZED;
        }

        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
        ///////////////////LOGGING\\\\\\\\\\\\\\\\\\\\
        //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\

        Logger.getInstance().recordOutput("StateSupervisor/EstimatedPose", robot_state.estimatedPose);
        Logger.getInstance().recordOutput("StateSupervisor/SkidAccumulator", robot_state.skidAccumulator.getNorm());
        Logger.getInstance().recordOutput("StateSupervisor/Vision-EstimatorConvergence", robot_state.visionToEstimatorDisagreement.get());
        Logger.getInstance().recordOutput("StateSupervisor/LocalizationStatus", robot_state.localizationStatus.toString());

    }

    //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
    ///////////////SKID DETECTION\\\\\\\\\\\\\\\\\
    //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
    /*
     * Adds to the accumulated skid error that is being stored as a chassis speed
     * @param measuredVelocity - the measurement of the chassis speed straight from the encoders
     * @param filteredVelocity - the measurement of the chassis speed after removing any detected instances of skid
     */
    public void addSkidMeasurement(ChassisSpeeds measuredVelocity, ChassisSpeeds filteredVelocity) {
        Translation2d t = robot_state.skidAccumulator;

        robot_state.skidAccumulator = 
            new Translation2d(
                t.getX() + (Math.abs(measuredVelocity.vxMetersPerSecond - filteredVelocity.vxMetersPerSecond) * Constants.kloop_period),
                t.getY() + (Math.abs(measuredVelocity.vyMetersPerSecond - filteredVelocity.vyMetersPerSecond) * Constants.kloop_period)
            );
    }

    public void addVisionCheck(Pose2d measuredPose) {
        //add the estimator vs. vision measurement to the moving average to determine convergence
        robot_state.visionToEstimatorDisagreement.add(measuredPose.minus(robot_state.estimatedPose).getTranslation().getNorm());
        //update general info
        robot_state.estimatedPose = pose_estimator.getEstimatedPosition();
    }

    private void clearSkidAccumulator() {
        robot_state.skidAccumulator = new Translation2d();
    }

    private void clearVisionEstimatorDisagreement() {
        robot_state.visionToEstimatorDisagreement.clear();
        robot_state.visionToEstimatorDisagreement.add(1.0);
    }

    public void setPose(Pose2d pose) {
        swerve.setPose(pose);
        robot_state.localizationStatus = LocalizationStatus.LOCALIZED;
        clearSkidAccumulator();
        clearVisionEstimatorDisagreement();
    }

    //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\
    /////////////OBJECTIVE TRACKER\\\\\\\\\\\\\\\\
    //////////////////////\\\\\\\\\\\\\\\\\\\\\\\\

    public int getColumn() {
        return objective_tracker.getColumn();
    }

    public NodeLevel getNodeLevel() {
        return objective_tracker.getNodeLeve();
    }

    public Command shiftNodeCommand(Direction direction) {
        return objective_tracker.shiftNodeCommand(direction);
    }

    public SuperstructureState getDesiredSuperstructureState() {
        NodeLevel selectedLevel = objective_tracker.getNodeLeve();
        int selectedColumn = objective_tracker.getColumn();

        switch(selectedLevel) {
            case HIGH:
                if(selectedColumn == 1 || selectedColumn == 4 || selectedColumn == 7)
                    return SuperstructureState.Preset.HIGH_CUBE.getState();
                return SuperstructureState.Preset.HIGH_CONE.getState();
            case MID:
                if(selectedColumn == 1 || selectedColumn == 4 || selectedColumn == 7)
                    return SuperstructureState.Preset.MID_CUBE.getState();
                return SuperstructureState.Preset.MID_CONE.getState();
            case HYBRID:
                return SuperstructureState.Preset.LOW_CUBE.getState();
        }
        return SuperstructureState.Preset.HOLDING.getState();
    }

    public SuperstructureState getDesiredHoldingSuperstructureState() {
        if(RobotContainer.manipulator.getGamePieceDetected().equals(GamePiece.CONE)) {
            return SuperstructureState.Preset.HOLDING_CONE.getState();
        } else if(RobotContainer.manipulator.getGamePieceDetected().equals(GamePiece.CUBE)) {
            return SuperstructureState.Preset.HOLDING_CUBE.getState();
        }
        return SuperstructureState.Preset.HOLDING_NADA.getState();
    }


    public RobotState getRobotState() {
        return robot_state;
    }

    public static class RobotState {
        //General Info
        public Pose2d estimatedPose;
        public LocalizationStatus localizationStatus;

        //Skid/Swerve Related
        public Translation2d skidAccumulator;

        //Vision Related
        //public ArrayList<VisionTarget> visionTargets;
        public MovingAverage visionToEstimatorDisagreement;
        
    }

    public enum LocalizationStatus {
        LOCALIZED,
        DELOCALIZED,
        LOCALIZING
    }
    
}