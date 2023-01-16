package org.team5557.state;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team5557.subsystems.swerve.SwerveSubsystemConstants.*;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.state.vision.VisionManager;
import org.team5557.state.vision.VisionTarget;
import org.team5557.subsystems.swerve.Swerve;

public class RobotStateSupervisor extends SubsystemBase {

    private RobotState robot_state = new RobotState();
    private final Swerve swerve = RobotContainer.swerve;
    private final VisionManager vision_manager = new VisionManager();
    //private final GoalPlanner goal_planner = new GoalPlanner();
    //private final SwerveDrivePoseEstimator estimator;

    public RobotStateSupervisor() {
        /*estimator = new SwerveDrivePoseEstimator(
            KINEMATICS, 
            RobotContainer.swerve.getGyroscopeAzimuth(), 
            DEFAULT_POSITIONS, 
            new Pose2d(),
            Constants.estimator.stateStdDevs, // estimator values (x, y, rotation) std-devs
            Constants.estimator.normalVisionStdDevs); // Vision (x, y, rotation) std-devs*/
            

    }


    @Override
    public void periodic() {
        //recieve list of vision measurements from VM
        robot_state.visionTargets = vision_manager.getTargets();
        //add them to the pose estimator
        for (VisionTarget target : robot_state.visionTargets) {
            RobotContainer.swerve.getEstimator().addVisionMeasurement(target.measuredPose, target.timestamp);
        }

        //recieve data from swerve

        //add data to pose estimator
        RobotContainer.swerve.getEstimator().update(swerve.getGyroscopeAzimuth(), swerve.getModulePositions());
    }
    


    public RobotState getRobotState() {
        return robot_state;
    }

    public static class RobotState {
        ArrayList<VisionTarget> visionTargets;
        Pose2d currentPose;
        LocalizationSatus localizationStatus;
    }
 
    /* POSES AND ESTIMATOR */
    /*
    public SwerveDrivePoseEstimator getEstimator() {
        return estimator;
    }*/

    /**
     * Returns the position of the robot
     */
    /*
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }*/

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void setPose(Pose2d pose) {
        //estimator.resetPosition(getGyroscopeAzimuth(), getModulePositions(), pose);
    }

    public enum LocalizationSatus {
        LOCALIZED,
    }
    
}
