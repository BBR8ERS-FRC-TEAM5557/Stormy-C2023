package org.team5557.subsystems.manipulator.util;


public class ManipulatorState {
    public double topRollerSpeed;
    public double bottomRollerSpeed;

    public ManipulatorState(double topRollerSpeed, double bottomRollerSpeed) {
        this.topRollerSpeed = topRollerSpeed;
        this.bottomRollerSpeed = bottomRollerSpeed;
    }

    public static enum ManipulatorStates {
        INTAKING_CUBE(new ManipulatorState(0.75, 0.75)),
        INTAKING_CUBE_SLOWED(new ManipulatorState(0.0, 0.75)),
        INTAKING_CUBE_REVERSED(new ManipulatorState(0.75, -0.75)),
        INTAKING_CONE(new ManipulatorState(-0.75, -0.75)),
        IDLE(new ManipulatorState(0.1, 0.1)),

        EJECT_CUBE(new ManipulatorState(-0.25, -0.25)),
        EJECT_CUBE_MID(new ManipulatorState(-0.9, -0.9)),
        EJECT_CUBE_HIGH(new ManipulatorState(-1.0, -1.0)),

        EJECT_CONE(new ManipulatorState(0.25, 0.25)),
        EJECT_CONE_FAST(new ManipulatorState(-0.75, -0.75)),
        DO_NOTHING(new ManipulatorState(0.0, 0.0));
        
        ManipulatorState state;
        private ManipulatorStates(ManipulatorState state) {
            this.state = state;
        }
        public ManipulatorState getManipulatorState() {
            return state;
        }
    }
}
