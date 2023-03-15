package org.team5557.subsystems.manipulator;

import static org.team5557.subsystems.manipulator.util.ManipulatorSubsystemConstants.kBottomRollerMotorID;
import static org.team5557.subsystems.manipulator.util.ManipulatorSubsystemConstants.kSubsystemID;
import static org.team5557.subsystems.manipulator.util.ManipulatorSubsystemConstants.kTopRollerMotorID;
import static org.team5557.subsystems.manipulator.util.ManipulatorSubsystemConstants.motorsInverted;

import org.library.team3061.util.CANDeviceFinder;
import org.library.team3061.util.CANDeviceId.CANDeviceType;
import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.subsystems.manipulator.util.ManipulatorState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax mTopRollerMotor;
    private final CANSparkMax mBottomRollerMotor;

    private final CANDeviceFinder can = new CANDeviceFinder();

    private ManipulatorState mWantedState = ManipulatorState.ManipulatorStates.DO_NOTHING.getManipulatorState();

    public Manipulator() {
        can.isDevicePresent(CANDeviceType.SPARK_MAX, kTopRollerMotorID.getDeviceNumber(), kSubsystemID + " Top Roller");
        can.isDevicePresent(CANDeviceType.SPARK_MAX, kBottomRollerMotorID.getDeviceNumber(), kSubsystemID + " Bottom Roller");

        mTopRollerMotor = new CANSparkMax(kTopRollerMotorID.getDeviceNumber(), MotorType.kBrushless);
        mTopRollerMotor.setIdleMode(IdleMode.kBrake);
        mTopRollerMotor.setInverted(motorsInverted);
        mTopRollerMotor.setSmartCurrentLimit(40, 40, 0);

        mBottomRollerMotor = new CANSparkMax(kBottomRollerMotorID.getDeviceNumber(), MotorType.kBrushless);
        mBottomRollerMotor.setIdleMode(IdleMode.kBrake);
        mBottomRollerMotor.setInverted(motorsInverted);
        mBottomRollerMotor.setSmartCurrentLimit(40, 40, 0);

        ShuffleboardTab tab = Shuffleboard.getTab(kSubsystemID);
        if(Constants.tuning_mode) {
            tab.add(this);
            tab.addNumber("Top Roller Speed", () -> mTopRollerMotor.getAppliedOutput());
            tab.addNumber("Bottom Roller Speed", () -> mBottomRollerMotor.getAppliedOutput());
        }
    }

    @Override
    public void periodic() {
        mTopRollerMotor.set(mWantedState.topRollerSpeed);
        mBottomRollerMotor.set(mWantedState.bottomRollerSpeed);

        Logger.getInstance().recordOutput(kSubsystemID + "/Top Roller/Applied Output", mTopRollerMotor.getAppliedOutput());

        Logger.getInstance().recordOutput(kSubsystemID + "/Bottom Roller/Applied Output", mBottomRollerMotor.getAppliedOutput());
    }

    public void setManipulatorState(ManipulatorState state) {
        mWantedState = state;
    }

    public ManipulatorState getManipulatorState() {
        return new ManipulatorState(mTopRollerMotor.getAppliedOutput(), mBottomRollerMotor.getAppliedOutput());
    }
}
