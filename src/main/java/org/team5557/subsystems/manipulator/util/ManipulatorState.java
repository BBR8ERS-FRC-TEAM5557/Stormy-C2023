package org.team5557.subsystems.manipulator.util;

import org.team5557.state.goal.ObjectiveTracker.GamePiece;

public class ManipulatorState {
    public double motorSpeed;
    public GamePiece observedPiece;

    public ManipulatorState(double motorSpeed, GamePiece observedPiece) {
        this.motorSpeed = motorSpeed;
        this.observedPiece = observedPiece;
    }

    public static enum ManipulatorStates {
        INTAKING_CUBE(new ManipulatorState(0.5, GamePiece.CUBE)),
        INTAKING_CONE(new ManipulatorState(0.5, GamePiece.CONE)),
        IDLE(new ManipulatorState(0.1, GamePiece.CONE));
        

        ManipulatorState state;
        private ManipulatorStates(ManipulatorState state) {
            this.state = state;
        }
        public ManipulatorState getManipulatorState() {
            return state;
        }
    }
}
