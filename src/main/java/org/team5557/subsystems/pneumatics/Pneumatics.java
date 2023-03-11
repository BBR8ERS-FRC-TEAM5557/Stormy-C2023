package org.team5557.subsystems.pneumatics;

import org.team5557.Constants;
import org.team5557.subsystems.pneumatics.util.PneumaticsSubsystemConstants;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private final PneumaticHub ph = new PneumaticHub(Constants.ports.ph);

    @Override
    public void periodic() {
        ph.enableCompressorAnalog(PneumaticsSubsystemConstants.kMinPressure, PneumaticsSubsystemConstants.kMaxPressure);
    }

    public PneumaticHub getPH() {
        return ph;
    }
}
