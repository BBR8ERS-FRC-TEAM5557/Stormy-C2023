package org.team5557.state.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

import org.photonvision.targeting.TargetCorner;

public class VisionTarget {
    public Pose2d measuredPose;
    public double timestamp;
    public MeasurementFidelity fidelity;
    public int cameraID;
    public List<TargetCorner> corners;

    public enum MeasurementFidelity {
        HIGH,
        MEDIUM,
        LOW
    }
}
