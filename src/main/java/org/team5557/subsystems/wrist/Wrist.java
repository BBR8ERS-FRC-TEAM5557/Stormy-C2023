package org.team5557.subsystems.wrist;

import org.library.team254.drivers.ServoMotorSubsystemAbs;
import org.library.team2910.math.MathUtils;
import org.team5557.RobotContainer;
import org.team5557.subsystems.wrist.util.WristConversions;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Wrist extends ServoMotorSubsystemAbs {

    public Wrist(ServoMotorSubsystemAbsConstants constants) {
        super(constants);

        ShuffleboardTab tab = Shuffleboard.getTab("Wrist");
        tab.add(this);
        tab.addNumber("Angle", () -> getAngleRelShoulder());

        mMasterEncoder.setPositionConversionFactor(360.0);
        mMasterEncoder.setZeroOffset(0.0);
    }
    
    // Syntactic sugar.
    public synchronized double getAngleRelShoulder() {
        return WristConversions.encoderRelToShoulderRel(getRawAngle(), RobotContainer.shoulder.getPosition());
    }

    public synchronized double getRawAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return MathUtils.epsilonEquals(mPeriodicIO.position_units, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }

    public double constrainUnits(double units) {
        return super.constrainUnits(units);
    }
}
