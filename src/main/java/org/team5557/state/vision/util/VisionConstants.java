package org.team5557.state.vision.util;

import org.library.team6328.util.PolynomialRegression;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
        public static final PolynomialRegression xyStdDevModel;
        public static final PolynomialRegression thetaStdDevModel;

        static {
                xyStdDevModel = new PolynomialRegression(
                                new double[] {
                                                0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358,
                                                2.758358,
                                                3.223358, 4.093358, 4.726358
                                },
                                new double[] {
                                                0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.0695, 0.046, 0.1245,
                                                0.0815, 0.193
                                },
                                1);
                thetaStdDevModel = new PolynomialRegression(
                                new double[] {
                                                0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358,
                                                2.758358,
                                                3.223358, 4.093358, 4.726358
                                },
                                new double[] {
                                                3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0, 4.0, 5.0,
                                                5.0
                                },
                                1);
        }

        public static final Transform3d kAnakinCameraToOrigin =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(-11.5), Units.inchesToMeters(-18.0)), //forward -/back +, left +/right -, up/down
                        new Rotation3d(0.0, Units.degreesToRadians(-6.0), Units.degreesToRadians(150.0)));

        public static final Transform3d kObiwanCameraToOrigin = 
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(11.5),  Units.inchesToMeters(-18.0)),
                        new Rotation3d(0.0, Units.degreesToRadians(-6.0), Units.degreesToRadians(210.0)));

}
