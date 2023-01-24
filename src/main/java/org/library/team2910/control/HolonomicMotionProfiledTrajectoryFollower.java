package org.library.team2910.control;

import org.library.team2910.math.RigidTransform2;
import org.library.team2910.math.Vector2;
import org.library.team2910.util.HolonomicDriveSignal;
import org.library.team2910.util.HolonomicFeedforward;

public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {
    private PidController forwardController;
    private PidController strafeController;
    private PidController rotationController;

    private HolonomicFeedforward feedforward;

    private Trajectory.State lastState = null;

    private boolean finished = false;

    public HolonomicMotionProfiledTrajectoryFollower(PidConstants translationConstants, PidConstants rotationConstants,
                                                     HolonomicFeedforward feedforward) {
        this.forwardController = new PidController(translationConstants);
        this.strafeController = new PidController(translationConstants);
        this.rotationController = new PidController(rotationConstants);
        this.rotationController.setContinuous(true);
        this.rotationController.setInputRange(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                               double rotationalVelocity, Trajectory trajectory, double time,
                                               double dt) {
        if (time > trajectory.getDuration()) {
            finished = true;
            return new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
        }   //once enough time has passed to complete the trajectory, finished = true and code sets the drive signal to nothing

        lastState = trajectory.calculate(time);

        Vector2 segmentVelocity = Vector2.fromAngle(lastState.getPathState().getHeading()).scale(lastState.getVelocity());
        Vector2 segmentAcceleration = Vector2.fromAngle(lastState.getPathState().getHeading()).scale(lastState.getAcceleration());

        Vector2 feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        forwardController.setSetpoint(lastState.getPathState().getPosition().x);
        strafeController.setSetpoint(lastState.getPathState().getPosition().y);
        rotationController.setSetpoint(lastState.getPathState().getRotation().toRadians());

        return new HolonomicDriveSignal(
                new Vector2(
                        forwardController.calculate(currentPose.translation.x, dt) + feedforwardVector.x,
                        strafeController.calculate(currentPose.translation.y, dt) + feedforwardVector.y
                ),
                rotationController.calculate(currentPose.rotation.toRadians(), dt),
                true
        );
    }

    public Trajectory.State getLastState() {
        return lastState;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();

        finished = false;
    }
}
