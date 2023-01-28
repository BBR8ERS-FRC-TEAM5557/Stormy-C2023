package org.team5557.state;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.RobotContainer;
import org.team5557.state.pathfind.Pathweaver;
import org.team5557.state.pathfind.util.Edge;
import org.team5557.state.pathfind.util.Node;
import org.team5557.state.pathfind.util.Obstacle;
import org.team5557.state.vision.VisionManager;
import org.team5557.state.vision.VisionTarget;
import org.team5557.subsystems.swerve.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.library.team2910.util.MovingAverage;
import org.littletonrobotics.junction.Logger;

public class RobotStateSupervisor extends SubsystemBase {

    private RobotState robot_state = new RobotState();
    private final Swerve swerve = RobotContainer.swerve;
    private final VisionManager vision_manager = new VisionManager();

    final List<Obstacle> obstacles = FieldConstants.obstacles;
    private final Pathweaver path_weaver = new Pathweaver(0, obstacles);


    //private final GoalPlanner goal_planner = new GoalPlanner();
    private final SwerveDrivePoseEstimator pose_estimator = swerve.getEstimator();

    private final GenericEntry skidDeaccumulationSensitivity;

    public RobotStateSupervisor() {
        configurePathWeaver();
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

    ////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
    ///////////////A* PATH GENERATOR\\\\\\\\\\\\\\\\\
    ////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\

    private void configurePathWeaver() {
        path_weaver.addNode(new Node(new Pose2d()));

        path_weaver.addNode(new Node(1,0));
        path_weaver.addNode(new Node(2.92-0.42,4.75));
        path_weaver.addNode(new Node(2.92-0.42,1.51-0.42));
        path_weaver.addNode(new Node(6,4.75));
        path_weaver.addNode(new Node(6,1.51-0.42));

        path_weaver.generateNodeEdges();
    }

    public List<Node> generateNodePath(Node start, Node end) {
        return path_weaver.findPath(start, end);
    }


    public PathPlannerTrajectory generatePath() {
        PathPlannerTrajectory trajectory;
        Node startPoint = new Node(swerve.getPose());
        Node endPoint = new Node(new Pose2d());

        List<Node> fullPath = this.generateNodePath(startPoint, endPoint);
        
        double startingSpeed = Math.hypot(swerve.getFilteredVelocity().vxMetersPerSecond, swerve.getFilteredVelocity().vyMetersPerSecond);
        Rotation2d heading = new Rotation2d(fullPath.get(1).getX() - startPoint.getX(), fullPath.get(1).getY() - startPoint.getY());
        if(startingSpeed > 0.05){
            heading = new Rotation2d(swerve.getFilteredVelocity().vxMetersPerSecond, swerve.getFilteredVelocity().vyMetersPerSecond);
        }
      
          // Depending on if internal points are present, make a new array of the other
          // points in the path.
          PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
       
          for (int i = 0; i < fullPath.size(); i++) {
            if (i == 0) {
              fullPathPoints[i] = new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
                  swerve.getPose().getRotation(), startingSpeed);
            } else if (i + 1 == fullPath.size()) {
              fullPathPoints[i] = new PathPoint(new Translation2d(endPoint.getX(), endPoint.getY()),
                  new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
                  endPoint.getHolRot());
            } else {
              fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
              new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
              endPoint.getHolRot());
            }
          }

          trajectory = PathPlanner.generatePath(new PathConstraints(2, 2), Arrays.asList(fullPathPoints));
          return trajectory;
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
        public Pose2d estimatedPose;
        public LocalizationStatus localizationStatus;

        //Skid/Swerve Related
        public Translation2d skidAccumulator;

        //Vision Related
        public ArrayList<VisionTarget> visionTargets;
        public MovingAverage visionToEstimatorDisagreement;
        
    }

    public enum LocalizationStatus {
        LOCALIZED,
        DELOCALIZED,
        LOCALIZING
    }
    
}