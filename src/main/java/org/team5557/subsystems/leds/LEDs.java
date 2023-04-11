package org.team5557.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private final CANdle mCandle = new CANdle(5);
    private double timestamp = 0.0;

    private final boolean mUseSmartdash = true; // if we want to manual control lights using shuffleboard

    private LEDStatus mElevatorBeamStatus = new LEDStatus(8, 75);

    // shuffleboard selectors

    private SendableChooser<State> mElevatorBeamChooser;

    // led states
    public enum State {
        OFF("OFF", -100, new ColorFlowAnimation(0, 0, 0)),
        RAINBOW("RAINBOW", 0, new RainbowAnimation(1.0, 1.0, 0)),
        BLUE_ALLIANCE("BLUE_ALLIANCE", 0, new LarsonAnimation(0, 0, 255, 0, 0.25, 0, BounceMode.Center, 4)),
        RED_ALLIANCE("RED_ALLIANCE", 0, new LarsonAnimation(255, 0, 0, 0, 0.25, 0, BounceMode.Center, 4)),
        IDLE("BASIC_BLUE", -1, new ColorFlowAnimation(0, 0, 255, 0, 0.25, 0, Direction.Forward)),

        PLEAD_FOR_CONE("YELLOW_FLASH", 1, new StrobeAnimation(250, 253, 15, 0, 0.25, 0)),
        PLEAD_FOR_CUBE("PURPLE_FLASH", 1, new StrobeAnimation(174, 55, 255, 0, 0.25, 0)),

        COPILOT_FOLLOWING("GREEN_FLASH", 2, new StrobeAnimation(0, 255, 50, 0, 0.25, 0)),
        COPILOT_AT_TARGET("GREEN_STROBE", 2, new StrobeAnimation(0, 255, 50, 0, 0.5, 0)),
        READY_TO_SCORE("SOLID_GREEN", 3, new ColorFlowAnimation(0, 255, 0, 0, 0.5, 0, Direction.Forward));

        String name; // name of state
        Animation animation;
        int priority;
        int duration;

        private State(String name, int priority, Animation animation) {
            this.name = name;
            this.priority = priority;
            this.animation = animation;
        }

        public String getName() {
            return name;
        }
    }

    public LEDs() {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        configureCandle(); // set CTRE configurations for CANdle

        // create sendable choosers for shuffleboard
        if (mUseSmartdash) {
            mElevatorBeamChooser = new SendableChooser<>();
            for (State state : State.values()) {
                mElevatorBeamChooser.addOption(state.getName(), state);
            }
            mElevatorBeamChooser.setDefaultOption("OFF", State.OFF);
            tab.add("Elevator LEDS", mElevatorBeamChooser).withSize(2, 1).withPosition(2, 0);
        }
    }

    @Override
    public void periodic() {
        outputTelemtry();
        timestamp = Timer.getFPGATimestamp(); // update timestamp for color cycling
        if (mUseSmartdash) { // pull states from smartdash
            requestStateFromDashboard(mElevatorBeamChooser.getSelected());
        }
        updateStates();
    }

    public void updateStates() {
        updateElevatorBeamLeds();
        //updateUnderglowLeds();
    }

    private void updateElevatorBeamLeds() {
        if (timestamp >= mElevatorBeamStatus.statusExpiryTime && !mUseSmartdash) {
            mElevatorBeamStatus.flush();
        }
        mElevatorBeamStatus.state.animation.setLedOffset(mElevatorBeamStatus.startIDx);
        mElevatorBeamStatus.state.animation.setNumLed(mElevatorBeamStatus.LEDCount);
        mCandle.animate(mElevatorBeamStatus.state.animation);
    }

    private void updateUnderglowLeds() {
        if (timestamp >= mElevatorBeamStatus.statusExpiryTime && !mUseSmartdash) {
            mElevatorBeamStatus.flush();
        }
        mElevatorBeamStatus.state.animation.setLedOffset(mElevatorBeamStatus.startIDx);
        mElevatorBeamStatus.state.animation.setNumLed(mElevatorBeamStatus.LEDCount);
        mCandle.animate(mElevatorBeamStatus.state.animation);
    }

    // setter functions
    public void requestState(State underglowState) {
        mElevatorBeamStatus.addRequest(underglowState);
        // this is a function for all of the states
    }

    // this is bad code just ignore that its here and live with it
    /**
     * Use this method when requesting a state from the dashboard
     * 
     * @param fromDashboard  - overides
     * @param underglowState
     */
    private void requestStateFromDashboard(State underglowState) {
        mElevatorBeamStatus.addDashboardRequest(underglowState);
    }

    /**
     * Adds a request to the underglow queu; it will be regulated by the priority of
     * the state
     * 
     * @param state - requested LED state
     */
    public void requestElevatorBeamState(State state) {
        mElevatorBeamStatus.addRequest(state);
    }

    // apply configuration to candle
    private void configureCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, 250);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
    }

    public void clearAnimation() {
        mCandle.clearAnimation(0);
    }

    // getter functions
    public State getElevatorBeamState() {
        return mElevatorBeamStatus.state;
    }

    public boolean getUsingSmartdash() {
        return mUseSmartdash;
    }

    private void outputTelemtry() {
        //if (Timer.getFPGATimestamp() % 100 == 0)
            
    }

    // class for holding information about each section
    private class LEDStatus {
        private State state = State.OFF; // current state
        private double statusExpiryTime = Timer.getFPGATimestamp(); // timestampe of last color cycle
        private int startIDx, LEDCount; // start and end of section

        public LEDStatus(int startIndex, int endIndex) {
            startIDx = startIndex;
            LEDCount = endIndex - startIndex;
        }

        public void addRequest(State request) {
            if (request.priority > state.priority && request != state) {
                state = request;
            }
            statusExpiryTime = Timer.getFPGATimestamp() + 0.3;
        }

        public void addDashboardRequest(State request) {
            state = request;
        }

        public void flush() {
            if (DriverStation.getAlliance() == Alliance.Red) {
                state = State.RED_ALLIANCE;
            } else {
                state = State.BLUE_ALLIANCE;
            }
        }
    }
}