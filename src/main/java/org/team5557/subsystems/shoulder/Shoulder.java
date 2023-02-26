package org.team5557.subsystems.shoulder;

import org.library.team254.drivers.ServoMotorSubsystemAbs;
import org.library.team2910.math.MathUtils;

public class Shoulder extends ServoMotorSubsystemAbs {

    public Shoulder(ServoMotorSubsystemAbsConstants constants) {
        super(constants);
        
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return MathUtils.epsilonEquals(mPeriodicIO.position_units, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }
    
}
