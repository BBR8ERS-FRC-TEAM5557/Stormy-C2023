package org.team5557.subsystems.swerve.commands;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.Swerve.DriveMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class SwerveAuto {
    private static double driveSpeed = 1.0;
    private static double threshold = 12.0;


    public static Command driveTillStationFlip() {
        double driveVelocity = Math.copySign(driveSpeed, RobotContainer.swerve.getGyroscopePitch());
        double startSign = Math.copySign(1.0, RobotContainer.swerve.getGyroscopePitch());
        double thresholdDegrees = Math.copySign(threshold, -startSign);


        return new RunCommand(() -> RobotContainer.swerve.drive(
            new ChassisSpeeds(driveVelocity, 0.0, 0.0), 
            DriveMode.OPEN_LOOP, 
            false, 
            Constants.superstructure.center_of_rotation), 
            RobotContainer.swerve)
            .until(() -> Math.abs(RobotContainer.swerve.getGyroscopePitch() - thresholdDegrees) < 2.0);
    }

    public static Command driveTillFlat() {
        double driveVelocity = Math.copySign(driveSpeed, -RobotContainer.swerve.getGyroscopePitch());

        return new RunCommand(() -> RobotContainer.swerve.drive(
            new ChassisSpeeds(driveVelocity, 0.0, 0.0), 
            DriveMode.OPEN_LOOP, 
            false, 
            Constants.superstructure.center_of_rotation), 
        RobotContainer.swerve)
        .until(() -> Math.abs(RobotContainer.swerve.getGyroscopePitch()) < 1.0);
    }
}
