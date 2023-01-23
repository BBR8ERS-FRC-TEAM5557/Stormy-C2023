package org.team5557.state.vision;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.team5557.Constants;
import org.team5557.FieldConstants;
import org.team5557.state.vision.VisionTarget.MeasurementFidelity;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.*;

public class VisionManager {
    //private final AprilTagFieldLayout tag_layout = new AprilTagFieldLayout(Path.of("asdf;lkajsdf.json"));
    private final PhotonCameraExtension photonCamera;
    private final List<PhotonCameraExtension> camera_list;
    private ArrayList<VisionTarget> visibleTargets;

    public VisionManager() {
        this.photonCamera = new PhotonCameraExtension("Arducam_OV9281_USB_Camera", new Transform3d());
        camera_list = Collections.unmodifiableList(
            List.of(
                photonCamera
            )
        );

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.vision_readout_key);
        tab.addCamera("Arducam", "Arducam", "http://10.55.57.105:5800", "http://10.29.10.11:5800")
            .withSize(3, 3)
            .withPosition(4, 0);
    }

    public void update() {
        int camID = 0;
        visibleTargets.clear();

        for (PhotonCameraExtension camera : camera_list) {
            var pipelineResult = camera.getLatestResult();
            if (!pipelineResult.equals(camera.getLastPipelineResult()) && pipelineResult.hasTargets()) {
              camera.setLastPipelineResult(pipelineResult);
              double imageCaptureTime = pipelineResult.getTimestampSeconds();
              var target = pipelineResult.getBestTarget();
              var fiducialId = target.getFiducialId();
              var targetPose = FieldConstants.aprilTags.get(fiducialId);//tag_layout.getTagPose(fiducialId);
              if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId <= 8){ //&& targetPose.isPresent()) {
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());//targetPose.get().transformBy(camToTarget.inverse());
                var visionMeasurement = camPose.transformBy(camera.CAMERA_TO_ROBOT);

                VisionTarget visionTarget = new VisionTarget();
                visionTarget.cameraID = camID;
                visionTarget.measuredPose = visionMeasurement.toPose2d();
                visionTarget.corners = target.getDetectedCorners();
                visionTarget.timestamp = imageCaptureTime;
                if(target.getPoseAmbiguity() <= .05 && visionTarget.measuredPose.getTranslation().getNorm() < Constants.estimator.max_high_accuracy_distance) {
                    visionTarget.fidelity = MeasurementFidelity.HIGH;
                } else {
                    visionTarget.fidelity = MeasurementFidelity.MEDIUM;
                }
                visibleTargets.add(camID, visionTarget);
              }
            }
            camID++;
        }
    }
    
    public ArrayList<VisionTarget> getTargets() {
        return visibleTargets;
    }
}
