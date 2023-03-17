package org.library.team2713;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import org.team5557.FieldConstants;

import lombok.SneakyThrows;

/**
 * Provides utilities for converting blue-side PathPlanner objects to red-side. Reflection is needed
 * due to private or protected fields within PathPlanner's API.
 *
 * <p>These transformations assume an absolute field origin on the blue alliance driver station, on
 * the scoring table side (away from the human player station).
 *
 * <p>+X is the direction from blue alliance driver station to red alliance driver station.
 *
 * <p>+Y is the direction from scoring table to human player station.
 */
public class PPReflectedTransform {
  private static Field deltaPosField;
  private static Field curveRadiusField;
  private static Constructor<PathPlannerTrajectory> constructor;

  // Reflection is expensive. Create declared objects at startup instead of per
  // state or per
  // trajectory.
  static {
    try {
      // Set all `.deltaPos` on PathPlannerState objects to be public
      deltaPosField = PathPlannerState.class.getDeclaredField("deltaPos");
      deltaPosField.setAccessible(true);

      // Set all `.curveRadius` on PathPlannerState objects to be public
      curveRadiusField = PathPlannerState.class.getDeclaredField("curveRadius");
      curveRadiusField.setAccessible(true);

      // Access the private constructor that builds a trajectory from states
      constructor =
          PathPlannerTrajectory.class.getDeclaredConstructor(
              List.class, List.class, StopEvent.class, StopEvent.class, boolean.class);
      constructor.setAccessible(true);
    } catch (NoSuchFieldException | SecurityException | NoSuchMethodException e) {
      System.err.println(
          "Could not access private fields via reflection in PathPlannerTrajectory.");
      e.printStackTrace(System.err);
    }
  }

  /**
   * Transforms a blue-side PathPlannerState to a red-side PathPlannerState. This should not need to
   * be called from outside this class.
   *
   * @param state The blue-side state of a blue-side trajectory
   * @return A new red-side state, or the same state if the DriverStation is set to blue.
   */
  @SneakyThrows
  private static PathPlannerState reflectiveTransformState(PathPlannerState state) {
    /* 
    PathPlannerState transformedState = new PathPlannerState();

    // Move it to the other side of the field, with an absolute origin on blue side
    // Mirror the X, keep the Y the same.
    Translation2d transformedTranslation =
        new Translation2d(
            FieldConstants.fieldLength - state.poseMeters.getX(), state.poseMeters.getY());

    // The instantaneous heading of the trajectory needs to be negated
    Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
    // The holonomic heading needs to be negated and rotated
    Rotation2d transformedHolonomicRotation =
        state.holonomicRotation.times(-1).plus(Rotation2d.fromDegrees(180));

    transformedState.timeSeconds = state.timeSeconds;
    // Negate the velocity. If traveling from community to mid field on blue, the +X
    // velocity is
    // positive. If doing so on red, the +X velocity is negative.
    transformedState.velocityMetersPerSecond = -state.velocityMetersPerSecond;
    transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
    transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
    transformedState.holonomicRotation = transformedHolonomicRotation;
    transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
    transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

    // transformedState.deltaPos = state.deltaPos;
    deltaPosField.set(transformedState, deltaPosField.get(state));

    // transformedState.curveRadius = -state.curveRadius;
    curveRadiusField.set(transformedState, (-1) * (Double) curveRadiusField.get(state));

    return transformedState;
    */
    PathPlannerState transformedState = new PathPlannerState();

    Translation2d transformedTranslation =
        new Translation2d(FieldConstants.fieldLength - state.poseMeters.getX(), state.poseMeters.getY());
    Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
    Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1);

    transformedState.timeSeconds = state.timeSeconds;
    transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
    transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
    transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
    transformedState.holonomicRotation = transformedHolonomicRotation;
    transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;

    //transformedState.curveRadius = -state.curveRadius;
    curveRadiusField.set(transformedState, (-1) * (Double) curveRadiusField.get(state));

    transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

    //transformedState.deltaPos = state.deltaPos;
    deltaPosField.set(transformedState, deltaPosField.get(state));

    return transformedState;
  }

  /**
   * Transforms a blue-side PathPlannerTrajectory to a red-side PathPlannerTrajectory. In the event
   * where this fails for any reason, an empty trajectory is returned so as not cause unpredictable
   * behavior.
   *
   * @param trajectory the blue-side trajectory to transform
   * @return the equivalent red-side trajectory
   */
  @SneakyThrows
  public static PathPlannerTrajectory reflectiveTransformTrajectory(
      PathPlannerTrajectory trajectory) {
    List<Trajectory.State> transformedStates = new ArrayList<>();

    try {
      // Convert all the trajectory states to red-side
      for (Trajectory.State s : trajectory.getStates()) {
        PathPlannerState state = (PathPlannerState) s;
        transformedStates.add(reflectiveTransformState(state));
      }

      // Call the now unhidden constructor
      return constructor.newInstance(
          transformedStates,
          trajectory.getMarkers(),
          trajectory.getStartStopEvent(),
          trajectory.getEndStopEvent(),
          trajectory.fromGUI);
    } catch (IllegalArgumentException
        | IllegalAccessException
        | InstantiationException
        | InvocationTargetException e) {
      // If this fails on the real field, return an empty trajectory instead of
      // crashing
      if (DriverStation.isFMSAttached()) {
        return new PathPlannerTrajectory();
      } else {
        // otherwise just crash
        throw e;
      }
    }
  }
}