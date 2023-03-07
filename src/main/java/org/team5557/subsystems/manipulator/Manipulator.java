package org.team5557.subsystems.manipulator;

import org.team5557.Constants;
import org.team5557.state.goal.ObjectiveTracker.GamePiece;
import org.team5557.subsystems.manipulator.util.ManipulatorState;
import static org.team5557.subsystems.manipulator.util.ManipulatorSubsystemConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final DoubleSolenoid mClawSolenoid;
    private final CANSparkMax mTopRollerMotor;
    private final CANSparkMax mBottomRollerMotor;

    private ManipulatorState mCurrentState;
    private ManipulatorState mWantedState;

    public Manipulator() {
        
        mClawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ports.clawSolenoidForward, Constants.ports.clawSolenoidReverse);
        mTopRollerMotor = new CANSparkMax(kTopRollerMotorID, MotorType.kBrushless);
        mBottomRollerMotor = new CANSparkMax(kBottomRollerMotorID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        mTopRollerMotor.set(mWantedState.motorSpeed);
        mBottomRollerMotor.set(mWantedState.motorSpeed);

        //if(mClawSolenoid.get() == Value.kForward && )

        mCurrentState = new ManipulatorState(0, mClawSolenoid.get() == Value.kForward ? GamePiece.CUBE : GamePiece.CONE);
    }

    public void setManipulatorState(ManipulatorState state) {
        mWantedState = state;
    }

    public ManipulatorState getManipulatorState() {
        return mCurrentState;
    }
}
