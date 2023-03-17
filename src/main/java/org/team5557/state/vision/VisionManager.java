package org.team5557.state.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import org.library.team6328.util.Alert;
import org.library.team6328.util.Alert.AlertType;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.state.vision.util.Limelight;
import org.team5557.state.vision.util.PhotonCameraExtension;
import org.team5557.state.vision.util.VisionUpdate;
import org.team5557.state.vision.util.Limelight.CamMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static org.team5557.state.vision.util.VisionConstants.*;

public class VisionManager {
    private final Limelight limelight;
    private final PhotonCameraExtension anakin;
    private final PhotonCameraExtension obi_wan;
    private final List<PhotonCameraExtension> camera_list;

    private Consumer<VisionUpdate> visionConsumer;
    private Consumer<Pose2d> disagreementConsumer;

    private final Alert anakinAlert;
    private final Alert obiwanAlert;


    public VisionManager() {
        this.limelight =  new Limelight("driver");

        this.anakin = new PhotonCameraExtension("Arducam_OV9281_Anakin", kAnakinCameraToOrigin);
        this.obi_wan = new PhotonCameraExtension("Arducam_OV9281_Obi_Wan", new Transform3d());
        camera_list = Collections.unmodifiableList(
                List.of(
                        anakin,
                        obi_wan
                    )
                );

        limelight.setCamMode(CamMode.DRIVER);

        anakinAlert = new Alert("Camera Anakin disconnected", AlertType.WARNING);
        obiwanAlert = new Alert("Camera Obiwan disconnected", AlertType.WARNING);

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.vision_readout_key);
        if (Constants.tuning_mode) {
            tab.addCamera("Anakin", "Arducam_OV9281_Anakin", "http://10.55.57.11:1184", "http://photonvision.local:1184")
                .withSize(2, 2)
                .withPosition(1, 0);

            tab.addCamera("ObiWan", "Arducam_OV9281_Obi_Wan", "http://10.55.57.11:1185", "http://photonvision.local:1185")
                .withSize(2, 2)
                .withPosition(4, 0);

            tab.addCamera("Limelight", "limelight", "http://10.55.57.13:5800")
                .withSize(3, 3);
        }

        ShuffleboardTab driver = Shuffleboard.getTab(Constants.shuffleboard.driver_readout_key);
        if (Constants.tuning_mode) {
            tab.addCamera("ll", "limelight-driver", "http://10.55.57.13:5800")
                .withSize(3, 3);
        }
    }

    public void update() {
        anakinAlert.set(!anakin.isConnected());
        obiwanAlert.set(!obi_wan.isConnected());

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