package org.library.team2713;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.library.team6328.util.Alert;
import org.library.team6328.util.Alert.AlertType;

public class AutoPath {
  private PathPlannerTrajectory blueTrajectory, redTrajectory;

  public AutoPath(String name, PathConstraints constraints) {
    try {
      blueTrajectory = PathPlanner.loadPath(name, constraints);
      redTrajectory = PPReflectedTransform.reflectiveTransformTrajectory(blueTrajectory);
    } catch (NullPointerException e) {
      Alert alert = new Alert(name + "'s file is not found. You're a scrub", AlertType.ERROR);
      alert.set(true);
      blueTrajectory = new PathPlannerTrajectory();
      redTrajectory = new PathPlannerTrajectory();
    }

  }

  public PathPlannerTrajectory getTrajectory() {
    return DriverStation.getAlliance() == Alliance.Blue
        ? this.blueTrajectory
        : this.redTrajectory;
  }

}
