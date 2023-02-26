package org.team5557.state.vision.util;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonCameraExtension extends PhotonCamera {
    private PhotonPipelineResult lastPipelineResult;
    public final Transform3d CAMERA_TO_ROBOT;

    public PhotonCameraExtension(NetworkTableInstance instance, String cameraName, Transform3d cameraToRobot) {
        super(instance, cameraName);
        this.CAMERA_TO_ROBOT = cameraToRobot;
    }

    public PhotonCameraExtension(String cameraName, Transform3d cameraToRobot){
        super(cameraName);
        this.CAMERA_TO_ROBOT = cameraToRobot;
    }

    public void setLastPipelineResult(PhotonPipelineResult result) {
        lastPipelineResult = result;
    }

    public PhotonPipelineResult getLastPipelineResult() {
        return lastPipelineResult;
    }

}