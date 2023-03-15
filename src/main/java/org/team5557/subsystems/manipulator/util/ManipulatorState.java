package org.team5557.subsystems.manipulator.util;


public class ManipulatorState {
    public double topRollerSpeed;
    public double bottomRollerSpeed;

    public ManipulatorState(double topRollerSpeed, double bottomRollerSpeed) {
        this.topRollerSpeed = topRollerSpeed;
        this.bottomRollerSpeed = bottomRollerSpeed;
    }

    public static enum ManipulatorStates {
        INTAKING_CUBE(new ManipulatorState(-0.5, -0.5)),
        INTAKING_CONE(new ManipulatorState(-0.5, -0.5)),
        IDLE(new ManipulatorState(0.1, 0.1)),
        EJECT_CUBE(new ManipulatorState(0.75, 0.75)),
        EJECT_CONE(new ManipulatorState(0.75, 0.75)),
        DO_NOTHING(new ManipulatorState(0.0, 0.75));
        
        ManipulatorState state;
        private ManipulatorStates(ManipulatorState state) {
            this.state = state;
        }
        public ManipulatorState getManipulatorState() {
            return state;
        }
    }
}
