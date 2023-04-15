package org.team5557.planners.superstructure.util;

public class SuperstructureState {
    public double elevator; // inches
    public double shoulder; // degrees

    public static enum Preset {
        HOLDING(new SuperstructureState(3.5, 258.0)),
        HOLDING_CONE(new SuperstructureState(3.5, 250.0)), 
        HOLDING_CUBE(new SuperstructureState(3.5, 258.0)), 
        HOLDING_NADA(new SuperstructureState(3.5, 244.0)),

        HIGH_CUBE(new SuperstructureState(58.0, 242.0)),
        MID_CUBE(new SuperstructureState(25.0, 240.0)),
        MID_CUBE_SHOOTER(new SuperstructureState(0.0, 230.0)),
        LOW_CUBE(new SuperstructureState(0.0, 240.0)),

        HIGH_CONE(new SuperstructureState(69.0, 183.0)),
        MID_CONE(new SuperstructureState(32.0, 220.0)),
        LOW_CONE(new SuperstructureState(0.0, 230.0)),

        INTAKING_CONE(new SuperstructureState(0.5, 194)),
        INTAKING_CONE_FOLLOW_THRU(new SuperstructureState(0.5, 177.0)),

        INTAKING_CUBE(new SuperstructureState(0.5, 185)),
        //INTAKING_CUBE_REVERSED(new SuperstructureState(0.5, 193.0)),
        //INTAKING_CUBE_FOLLOW_THRU(new SuperstructureState(0.5, 190.0)),

        INTAKING_CHUTE_CONE(new SuperstructureState(0.5, 260.0));

        
        private SuperstructureState state;
        private Preset(SuperstructureState state) {
            this.state = state;
        }
        public SuperstructureState getState() {
            return state;
        }
    }



    public SuperstructureState(double elevator, double shoulder) {
        this.elevator = elevator;
        this.shoulder = shoulder;
    }

    public SuperstructureState(SuperstructureState other) {
        this.elevator = other.elevator;
        this.shoulder = other.shoulder;
    }

    // default robot position
    public SuperstructureState() {
        this(0, 0);
    }

    public void setFrom(SuperstructureState source) {
        elevator = source.elevator;
        shoulder = source.shoulder;
    }

    public boolean equals(SuperstructureState that) {
        return this.elevator == that.elevator &&
                this.shoulder == that.shoulder;
    }

    public boolean isAtDesiredState(SuperstructureState currentState) {
        double[] distances = {
                currentState.elevator - this.elevator,
                currentState.shoulder - this.shoulder,
        };
        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i]) > SuperstructureConstants.kPadding[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * @return height of the bottom roller of the end effector; only applicable for SFR and SVR end effectors
     
    public double getBottomEndEffectorHeight() {
        double z = Constants.kElevatorHeightToFloor; // z will represent the height of the bottom of the end effector to the ground in inches
        z += this.elevator;
        z += Constants.kArmLength * Math.sin(Math.toRadians(this.shoulder));
        z += Constants.kWristToBottomEndEffectorLength * Math.sin(Math.toRadians(this.wrist + Constants.kEndEffectorBottomAngle));
        return z;
    }*/

    /**
     * @return Translation2d where x maps to x position and y maps to z position
     
    public Translation2d getPlanarWristJointLocation() {
        double z = this.elevator;
        z += Constants.kArmLength * Math.sin(Math.toRadians(this.shoulder));

        double x = Constants.kArmLength * Math.cos(Math.toRadians(this.shoulder));

        return new Translation2d(x, z);
    }*/

    /**
     * @return is bottom roller of the end effector above the bumper; only applicable for SFR and SVR end effectors
     
    public boolean isOverBumper() {
        return getBottomEndEffectorHeight() > Constants.kBumperHeight + SuperstructureConstants.kElevatorPaddingInches;
    }*/

    @Override
    public String toString() {
        return "SuperstructureState{" +
                ", elevator=" + elevator +
                ", shoulder=" + shoulder +
                '}';
    }

    public Double[] asVector() {
        return new Double[]{elevator, shoulder};
    }
}