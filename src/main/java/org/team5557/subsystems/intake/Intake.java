package org.team5557.subsystems.intake;

import static org.team5557.subsystems.intake.util.IntakeSubsystemConstants.*;

import org.library.team3061.util.CANDeviceFinder;
import org.library.team3061.util.CANDeviceId.CANDeviceType;
import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.util.IntakeState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid solenoid;
    private final CANSparkMax motor;
    private final DigitalInput intake_gate;

    private final CANDeviceFinder can = new CANDeviceFinder();

    private IntakeState mWantedState = IntakeState.IntakeStates.DO_NOTHING.getIntakeState();
    private IntakeState mCurrentState = IntakeState.IntakeStates.DO_NOTHING.getIntakeState();

    public Intake() {
        can.isDevicePresent(CANDeviceType.SPARK_MAX, motorID.getDeviceNumber(), kSubsystemID);

        motor = new CANSparkMax(motorID.getDeviceNumber(), MotorType.kBrushless);
        motor.setOpenLoopRampRate(0.5);

        solenoid = RobotContainer.pneumatics.getPH().makeDoubleSolenoid(kSolenoidForward, kSolenoidReverse);

        intake_gate = new DigitalInput(kProximitySwitch);

        ShuffleboardTab tab = Shuffleboard.getTab(kSubsystemID);
        if(Constants.tuning_mode) {
            tab.add(this);
            tab.addBoolean("Cube Detected", this::getCubeDetected);
            tab.addBoolean("Intake Deployed", () -> getIntakeState().deployed);
            tab.addNumber("Intake Speed", () -> getIntakeState().motorSpeed);
        }

        ShuffleboardTab driver = Shuffleboard.getTab(Constants.shuffleboard.driver_readout_key);
        driver.addBoolean("Cube Detected", this::getCubeDetected);
    }

    @Override
    public void periodic() {
        motor.set(mWantedState.motorSpeed);

        
        if(mWantedState.deployed) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.set(Value.kReverse);
        }

        mCurrentState = new IntakeState(motor.getAppliedOutput(), solenoid.get() == Value.kForward);

        Logger.getInstance().recordOutput(kSubsystemID + "/Deployed?", mCurrentState.deployed);
        Logger.getInstance().recordOutput(kSubsystemID + "/Applied Output", mCurrentState.motorSpeed);
        Logger.getInstance().recordOutput(kSubsystemID + "/Cube Detected?", getCubeDetected());
    }

    public void setIntakeState(IntakeState state) {
        mWantedState = state;
    }

    public IntakeState getIntakeState() {
        return mCurrentState;
    }

    public synchronized boolean getCubeDetected() {
        return !intake_gate.get();
    }
}
