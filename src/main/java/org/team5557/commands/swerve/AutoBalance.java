package org.team5557.commands.swerve;

import org.library.team254.util.LatchedBoolean;
import org.library.team2910.math.MathUtils;
import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.Swerve.DriveMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
    Swerve swerve = RobotContainer.swerve;
    PIDController pid = new PIDController(0.001, 0, 0);
    Timer t = new Timer();
    LatchedBoolean latch = new LatchedBoolean();
    
    public AutoBalance() {
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pid.reset();

        t.stop();
        t.reset();
    }

    @Override
    public void execute() {
        double output = pid.calculate(swerve.getGyroscopePitch(), 0);
        MathUtils.clamp(output, -0.4, 0.4);
        double error = pid.getPositionError();
        boolean atSetpoint = Math.abs(error) < 3.0;
        

        if(latch.update(atSetpoint)) {
            t.start();
        }
        if(!atSetpoint) {
            t.stop();
            t.reset();
        }
        
        if (t.get() > 1.0) 
            swerve.drive(new ChassisSpeeds(), DriveMode.X_OUT, false, Constants.superstructure.center_of_rotation);
        else 
            swerve.drive(new ChassisSpeeds(-output, 0, 0), DriveMode.OPEN_LOOP, false, Constants.superstructure.center_of_rotation);


        Logger.getInstance().recordOutput("AutoBalance/output", output);
        Logger.getInstance().recordOutput("AutoBalance/error", pid.getPositionError());
    }

    
}
