package org.team5557.subsystems.intake;

import org.team5557.RobotContainer;
import org.team5557.subsystems.intake.util.IntakeState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid solenoid;
    private final CANSparkMax motor;

    private IntakeState mWantedState = IntakeState.IntakeStates.DO_NOTHING.getIntakeState();
    private IntakeState mCurrentState;

    public Intake() {
        motor = new CANSparkMax(60, MotorType.kBrushless);
        motor.setOpenLoopRampRate(0.5);
        solenoid = RobotContainer.pneumatics.getPH().makeDoubleSolenoid(0, 0);
    }

    @Override
    public void periodic() {
        motor.set(mWantedState.motorSpeed);

        if(mWantedState.deployed) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.set(Value.kReverse);
        }

        mCurrentState = new IntakeState(0, solenoid.get() == Value.kForward);
    }

    public void setIntakeState(IntakeState state) {
        mWantedState = state;
    }

    public IntakeState getIntakeState() {
        return mCurrentState;
    }
}
