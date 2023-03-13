package org.team5557.planners.arm;

import org.team5557.Constants;
import org.team5557.subsystems.shoulder.util.ShoulderSubsystemConstants;
import org.team5557.subsystems.wrist.util.WristSubsystemConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Converts between the system state and motor voltages for a double jointed
 * arm.
 *
 * <p>
 * https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 *
 * <p>
 * https://www.chiefdelphi.com/t/double-jointed-arm-physics-control-simulator/424307
 */
public class ArmDynamics {
        private static final double g = 9.80665;
        private final Translation2d origin;
        private final JointConfig shoulder;
        private final JointConfig wrist;

        public ArmDynamics() {
                this.origin = new Translation2d();
                this.shoulder = new JointConfig(ShoulderSubsystemConstants.kShoulderJointConstants.mass,
                                ShoulderSubsystemConstants.kShoulderJointConstants.length,
                                ShoulderSubsystemConstants.kShoulderJointConstants.moi,
                                ShoulderSubsystemConstants.kShoulderJointConstants.cgRadius,
                                ShoulderSubsystemConstants.kShoulderJointConstants.minAngle,
                                ShoulderSubsystemConstants.kShoulderJointConstants.maxAngle,
                                ShoulderSubsystemConstants.kShoulderJointConstants.reduction,
                                DCMotor.getNEO(1).withReduction(
                                                ShoulderSubsystemConstants.kShoulderJointConstants.reduction));

                this.wrist = new JointConfig(WristSubsystemConstants.kWristJointConstants.mass,
                                WristSubsystemConstants.kWristJointConstants.length,
                                WristSubsystemConstants.kWristJointConstants.moi,
                                WristSubsystemConstants.kWristJointConstants.cgRadius,
                                WristSubsystemConstants.kWristJointConstants.minAngle,
                                WristSubsystemConstants.kWristJointConstants.maxAngle,
                                WristSubsystemConstants.kWristJointConstants.reduction,
                                DCMotor.getNEO(1)
                                                .withReduction(WristSubsystemConstants.kWristJointConstants.reduction));
        }

        /** Calculates the joint voltages based on the joint positions (feedforward). */
        public Vector<N2> feedforward(Vector<N2> position) {
                return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
        }

        /**
         * Calculates the joint voltages based on the full joint states as a matrix
         * (feedforward). The
         * rows represent each joint and the columns represent position, velocity, and
         * acceleration.
         */
        public Vector<N2> feedforward(Matrix<N2, N3> state) {
                return feedforward(
                                new Vector<>(state.extractColumnVector(0)),
                                new Vector<>(state.extractColumnVector(1)),
                                new Vector<>(state.extractColumnVector(2)));
        }

        /**
         * Calculates the joint voltages based on the full joint states as vectors
         * (feedforward).
         */
        public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
                var torque = M(position)
                                .times(acceleration)
                                .plus(C(position, velocity).times(velocity))
                                .plus(Tg(position));
                return VecBuilder.fill(
                                shoulder.motor.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
                                wrist.motor.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
        }

        private Matrix<N2, N2> M(Vector<N2> position) {
                var M = new Matrix<>(N2.instance, N2.instance);
                M.set(
                                0,
                                0,
                                shoulder.mass * Math.pow(shoulder.cgRadius, 2.0)
                                                + wrist.mass * (Math.pow(shoulder.length, 2.0)
                                                                + Math.pow(wrist.cgRadius, 2.0))
                                                + shoulder.moi
                                                + wrist.moi
                                                + 2
                                                                * wrist.mass
                                                                * shoulder.length
                                                                * wrist.cgRadius
                                                                * Math.cos(position.get(1, 0)));
                M.set(
                                1,
                                0,
                                wrist.mass * Math.pow(wrist.cgRadius, 2.0)
                                                + wrist.moi
                                                + wrist.mass * shoulder.length * wrist.cgRadius
                                                                * Math.cos(position.get(1, 0)));
                M.set(
                                0,
                                1,
                                wrist.mass * Math.pow(wrist.cgRadius, 2.0)
                                                + wrist.moi
                                                + wrist.mass * shoulder.length * wrist.cgRadius
                                                                * Math.cos(position.get(1, 0)));
                M.set(1, 1, wrist.mass * Math.pow(wrist.cgRadius, 2.0) + wrist.moi);
                return M;
        }

        private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
                var C = new Matrix<>(N2.instance, N2.instance);
                C.set(
                                0,
                                0,
                                -wrist.mass
                                                * shoulder.length
                                                * wrist.cgRadius
                                                * Math.sin(position.get(1, 0))
                                                * velocity.get(1, 0));
                C.set(
                                1,
                                0,
                                wrist.mass
                                                * shoulder.length
                                                * wrist.cgRadius
                                                * Math.sin(position.get(1, 0))
                                                * velocity.get(0, 0));
                C.set(
                                0,
                                1,
                                -wrist.mass
                                                * shoulder.length
                                                * wrist.cgRadius
                                                * Math.sin(position.get(1, 0))
                                                * (velocity.get(0, 0) + velocity.get(1, 0)));
                return C;
        }

        private Matrix<N2, N1> Tg(Vector<N2> position) {
                var Tg = new Matrix<>(N2.instance, N1.instance);
                Tg.set(
                                0,
                                0,
                                (shoulder.mass * shoulder.cgRadius + wrist.mass * shoulder.length)
                                                * g
                                                * Math.cos(position.get(0, 0))
                                                + wrist.mass
                                                                * wrist.cgRadius
                                                                * g
                                                                * Math.cos(position.get(0, 0) + position.get(1, 0)));
                Tg.set(
                                1,
                                0,
                                wrist.mass * wrist.cgRadius * g * Math.cos(position.get(0, 0) + position.get(1, 0)));
                return Tg;
        }
}