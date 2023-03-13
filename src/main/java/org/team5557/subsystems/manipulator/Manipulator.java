package org.team5557.subsystems.manipulator;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.state.goal.ObjectiveTracker.GamePiece;
import org.team5557.subsystems.manipulator.util.ManipulatorState;
import static org.team5557.subsystems.manipulator.util.ManipulatorSubsystemConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final DoubleSolenoid mClawSolenoid;
    private final CANSparkMax mTopRollerMotor;
    private final CANSparkMax mBottomRollerMotor;

    private ManipulatorState mCurrentState;
    private ManipulatorState mWantedState = ManipulatorState.ManipulatorStates.DO_NOTHING.getManipulatorState();

    public Manipulator() {
        mClawSolenoid = RobotContainer.pneumatics.getPH().makeDoubleSolenoid(0, 0);

        mTopRollerMotor = new CANSparkMax(kTopRollerMotorID, MotorType.kBrushless);
        mTopRollerMotor.setInverted(motorsInverted);
        mTopRollerMotor.setSmartCurrentLimit(40, 40, 0);

        mBottomRollerMotor = new CANSparkMax(kBottomRollerMotorID, MotorType.kBrushless);
        mBottomRollerMotor.setInverted(!motorsInverted);
        mBottomRollerMotor.setSmartCurrentLimit(40, 40, 0);
    }

    @Override
    public void periodic() {
        mTopRollerMotor.set(mWantedState.motorSpeed);
        mBottomRollerMotor.set(mWantedState.motorSpeed);

        if(mWantedState.observedPiece == GamePiece.CUBE) {
            mClawSolenoid.set(Value.kForward);
        } else {
            mClawSolenoid.set(Value.kReverse);
        }

        mCurrentState = new ManipulatorState(0, mClawSolenoid.get() == Value.kForward ? GamePiece.CUBE : GamePiece.CONE);
    }

    public void setManipulatorState(ManipulatorState state) {
        mWantedState = state;
    }

    public ManipulatorState getManipulatorState() {
        return mCurrentState;
    }
}
