package org.team5557.subsystems.pneumatics;

import org.team5557.Constants;
import org.team5557.subsystems.pneumatics.util.PneumaticsSubsystemConstants;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private final PneumaticHub ph = new PneumaticHub(Constants.ports.ph);

    ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.driver_readout_key);

    public Pneumatics() {
        tab.addNumber("Pressure", () -> ph.getPressure(0));
    }

    @Override
    public void periodic() {
        ph.enableCompressorAnalog(PneumaticsSubsystemConstants.kMinPressure, PneumaticsSubsystemConstants.kMaxPressure);
    }

    public PneumaticHub getPH() {
        return ph;
    }
}
