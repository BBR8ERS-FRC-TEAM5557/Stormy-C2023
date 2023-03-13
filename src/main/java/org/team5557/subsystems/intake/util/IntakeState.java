package org.team5557.subsystems.intake.util;

import org.team5557.state.goal.ObjectiveTracker.GamePiece;

public class IntakeState {
    public double motorSpeed;
    public boolean deployed;

    public IntakeState(double motorSpeed, boolean deployed) {
        this.motorSpeed = motorSpeed;
        this.deployed = deployed;
    }

    public static enum IntakeStates {
        INTAKING_CUBE(new IntakeState(-0.75, true)),
        EJECT_CUBE(new IntakeState(0.5, true)),
        DO_NOTHING(new IntakeState(0.0, false));
        

        IntakeState state;
        private IntakeStates(IntakeState state) {
            this.state = state;
        }
        public IntakeState getIntakeState() {
            return state;
        }
    }
}