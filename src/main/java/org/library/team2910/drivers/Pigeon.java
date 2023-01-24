package org.library.team2910.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;

import org.library.team2910.math.Rotation2;


public class Pigeon extends Gyroscope {
    private final PigeonIMU handle;

    public Pigeon(int id) {
        this.handle = new PigeonIMU(id);
    }

    @Override
    public void calibrate() {
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromDegrees(handle.getFusedHeading());
    }

    @Override
    public double getUnadjustedRate() {
        return 0.0;
    }
}
