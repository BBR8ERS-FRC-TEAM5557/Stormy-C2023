package org.team5557.subsystems.elevator.commands;

import org.team5557.RobotContainer;
import org.team5557.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HomeElevator extends CommandBase {

    private final Elevator elevator = RobotContainer.elevator;

    private final double homingSpeed = -0.05;
    private final double timeoutS = 10.0;

    private final Timer t = new Timer();

    public HomeElevator() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //set leds and shit
        t.reset();
        t.start();

        elevator.disableSoftLimits();
    }

    @Override
    public void execute() {
        //make sure the superstructure is ready to have the elevator home

        //home the elevator

        elevator.setOpenLoop(homingSpeed);
    }

    @Override
    public boolean isFinished() {
        if(t.hasElapsed(timeoutS)) {
            System.out.println("Elevator Homing Timeout!!");
            finishHoming();
            return true;
        }
            
        if(!elevator.atHomingLocation())
            return false;

        finishHoming();
        return true;
    }

    public void finishHoming() {
        t.stop();
        t.reset();
        elevator.setOpenLoop(0.0);
        elevator.zeroSensors();
        elevator.enableSoftLimits();
        elevator.setHomed(true);
    }
}
