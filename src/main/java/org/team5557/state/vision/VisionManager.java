package org.team5557.state.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import org.library.team6328.util.PolynomialRegression;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.state.vision.util.PhotonCameraExtension;
import org.team5557.state.vision.util.VisionUpdate;
import org.team5557.state.vision.util.VisionUpdate.MeasurementFidelity;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VisionManager {
    private final PhotonCameraExtension anakin;
    private final PhotonCameraExtension obi_wan;
    private final List<PhotonCameraExtension> camera_list;

    private Consumer<VisionUpdate> visionConsumer;
    private Consumer<Pose2d> disagreementConsumer;

    private static final PolynomialRegression xyStdDevModel;
    private static final PolynomialRegression thetaStdDevModel;

    static {
        xyStdDevModel = new PolynomialRegression(
            new double[] {
                    0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                    3.223358, 4.093358, 4.726358
            },
            new double[] {
                    0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.0695, 0.046, 0.1245, 0.0815, 0.193
            },
            1);
        thetaStdDevModel = new PolynomialRegression(
            new double[] {
                    0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                    3.223358, 4.093358, 4.726358
            },
            new double[] {
                    0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068
            },
            1);
    }

    public VisionManager() {
        this.anakin = new PhotonCameraExtension("Arducam_OV9281_Anakin", new Transform3d());
        this.obi_wan = new PhotonCameraExtension("Arducam_OV9281_Obi_Wan", new Transform3d());
        camera_list = Collections.unmodifiableList(
                List.of(
                        anakin,
                        obi_wan
                    )
                );

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.vision_readout_key);
        if (Constants.tuning_mode) {
            tab.addCamera("Anakin", "Arducam_OV9281_Anakin", "http://10.55.57.11:1184", "http://photonvision.local:1184")
                .withSize(3, 3)
                .withPosition(4, 0);

            tab.addCamera("ObiWan", "Arducam_OV9281_Obi_Wan", "http://10.55.57.11:1185", "http://photonvision.local:1185")
                .withSize(3, 3)
                .withPosition(5, 0);
        }
    }

    public void update() {
        for (PhotonCameraExtension camera : camera_list) {
            var pipelineResult = camera.getLatestResult();

            if (!pipelineResult.equals(camera.getLastPipelineResult())) {
                camera.setLastPipelineResult(pipelineResult);

                double imageCaptureTime = pipelineResult.getTimestampSeconds();
                Logger.getInstance()
                        .recordOutput(
                                "Vision/" + camera.getName() + "/LatencySecs",
                                Timer.getFPGATimestamp() - imageCaptureTime);
            }

            if (pipelineResult.hasTargets()) {
                List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
                List<Pose2d> visionPose2ds = new ArrayList<>();
                List<Pose3d> tagPose3ds = new ArrayList<>();
                List<Integer> tagIds = new ArrayList<>();

                for (PhotonTrackedTarget target : targets) {
                    int fiducialId = target.getFiducialId();
                    Transform3d camToTarget = target.getBestCameraToTarget();
                    var fieldToTag = FieldConstants.aprilTags.get(fiducialId);

                    if (fieldToTag == null) {
                        continue;
                    }

                    Pose3d camPose = fieldToTag.transformBy(camToTarget.inverse());
                    Pose2d visionMeasurement = camPose.transformBy(camera.CAMERA_TO_ROBOT).toPose2d();

                    visionPose2ds.add(visionMeasurement);
                    tagPose3ds.add(camPose);
                    tagIds.add(target.getFiducialId());

                    if (target.getPoseAmbiguity() <= .2) {
                        double tagDistance = camPose.getTranslation().getNorm();
                        double xyStdDev = xyStdDevModel.predict(tagDistance);
                        double thetaStdDev = thetaStdDevModel.predict(tagDistance);
                        VisionUpdate visionUpdate = new VisionUpdate();

                        visionUpdate.measuredPose = visionMeasurement;
                        visionUpdate.corners = target.getDetectedCorners();
                        visionUpdate.timestamp = pipelineResult.getTimestampSeconds();
                        visionUpdate.stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

                        visionConsumer.accept(visionUpdate);
                        disagreementConsumer.accept(visionMeasurement);
                    }

                    Logger.getInstance()
                            .recordOutput(
                                    "Vision/" + camera.getName() + "/RobotPoses",
                                    visionPose2ds.toArray(new Pose2d[visionPose2ds.size()]));
                    Logger.getInstance()
                            .recordOutput(
                                    "Vision/" + camera.getName() + "/TagPoses",
                                    tagPose3ds.toArray(new Pose3d[tagPose3ds.size()]));
                    Logger.getInstance()
                            .recordOutput(
                                    "Vision/" + camera.getName() + "/TagIDs",
                                    tagIds.stream().mapToLong(Long::valueOf).toArray());
                    Logger.getInstance()
                            .recordOutput(
                                    "Vision/" + camera.getName() + "/PoseAmbiguity",
                                    target.getPoseAmbiguity());
                }
            }
        }
    }
    public void setDataInterface(Consumer<VisionUpdate> visionConsumer, Consumer<Pose2d> disagreementConsumer) {
        this.visionConsumer = visionConsumer;
        this.disagreementConsumer = disagreementConsumer;
    }
}