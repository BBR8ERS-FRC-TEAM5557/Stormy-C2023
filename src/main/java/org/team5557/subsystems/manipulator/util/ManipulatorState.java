package org.team5557.subsystems.manipulator.util;


public class ManipulatorState {
    public double topRollerSpeed;
    public double bottomRollerSpeed;

    public ManipulatorState(double topRollerSpeed, double bottomRollerSpeed) {
        this.topRollerSpeed = topRollerSpeed;
        this.bottomRollerSpeed = bottomRollerSpeed;
    }

    public static enum ManipulatorStates {
        //TOP ROLLER REFFERS TO THE INDEPENDENT CONE ROLLER
        //BOTTOM ROLLER REFFERS TO THE CUBE ROLLERS
        INTAKING_CUBE(new ManipulatorState(0.75, -0.75)),
        HOLDING_CUBE(new ManipulatorState(0.0, -0.05)),
        EJECT_CUBE(new ManipulatorState(0.0, 0.25)),


        INTAKING_CONE(new ManipulatorState(0.75, 0.75)),
        HOLDING_CONE(new ManipulatorState(0.05, 0.05)),
        EJECT_CONE(new ManipulatorState(0.15, 0.15)),


        IDLE(new ManipulatorState(0.1, 0.1)),
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
