package org.team5557.subsystems.shoulder;

import org.library.team254.drivers.ServoMotorSubsystemAbs;
import org.library.team254.motion.MotionProfileConstraints;
import org.library.team2910.math.MathUtils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shoulder extends ServoMotorSubsystemAbs {

    public Shoulder(ServoMotorSubsystemAbsConstants constants) {
        super(constants);

        ShuffleboardTab tab = Shuffleboard.getTab("Shoulder");
        tab.add(this);
        tab.addNumber("Angle", () -> getAngle());

        mMasterEncoder.setPositionConversionFactor(360.0);
        mMasterEncoder.setZeroOffset(14.0);
        mMotionProfileConstraints = new MotionProfileConstraints(constants.kCruiseVelocity, -constants.kCruiseVelocity, constants.kAcceleration);
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return MathUtils.epsilonEquals(mPeriodicIO.position_units, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }
    
}
