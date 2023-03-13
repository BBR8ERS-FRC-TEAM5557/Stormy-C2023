package org.team5557.state.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.List;

import org.photonvision.targeting.TargetCorner;

public class VisionUpdate {
    public Pose2d measuredPose;
    public double timestamp;
    public MeasurementFidelity fidelity;
    public int cameraID;
    public List<TargetCorner> corners;
    public Matrix<N3, N1> stdDevs;

    public enum MeasurementFidelity {
        HIGH,
        MEDIUM,
        LOW
    }
}