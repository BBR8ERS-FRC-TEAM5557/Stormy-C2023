package org.team5557.subsystems.swerve.commands;

import java.util.function.DoubleSupplier;

import org.team5557.Constants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.state.RobotStateSupervisor;
import org.team5557.state.goal.ObjectiveTracker.GamePiece;
import org.team5557.state.vision.util.DetectedObject;
import org.team5557.subsystems.swerve.Swerve;
import org.team5557.subsystems.swerve.Swerve.DriveMode;
import org.team5557.subsystems.swerve.util.RawControllers;
import org.team5557.subsystems.swerve.util.SwerveSubsystemConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoPickup extends CommandBase {
    private final XboxController primary_Controller;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier throttleSupplier;

    private final Swerve swerve;
    private final RobotStateSupervisor state;
    private final RawControllers rawControllers;
    private final boolean isCone;
    private double desAngle;

    public AutoPickup(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier throttleSupplier, boolean isCone) {
        this.swerve = RobotContainer.swerve;
        this.state = RobotContainer.state_supervisor;
        this.rawControllers = RobotContainer.raw_controllers;
        this.primary_Controller = RobotContainer.primary_controller;

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.throttleSupplier = throttleSupplier;

        this.isCone = isCone;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        desAngle = state.getRobotState().estimatedPose.getRotation().getRadians();
    }

    @Override
    public void execute() {
        DetectedObject update = state.getDetectedObject();
        ChassisSpeeds output;

        if(update.gamePiece != GamePiece.NONE)
            desAngle = state.getRobotState().estimatedPose.getRotation().plus(Rotation2d.fromRadians(-update.angleOffset)).getRadians();

        if(isCone && update.gamePiece == GamePiece.CONE) {
            swerve.drive(
                new ChassisSpeeds(
                    -throttleSupplier.getAsDouble() / SwerveSubsystemConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    0.0, 
                    rawControllers.calculateTheta(desAngle)
                ),
                DriveMode.OPEN_LOOP, 
                false, 
                Constants.superstructure.center_of_rotation
            );
        }
        else if(!isCone && update.gamePiece == GamePiece.CUBE) {
            swerve.drive(
                new ChassisSpeeds(
                    -throttleSupplier.getAsDouble(),
                    0.0, 
                    rawControllers.calculateTheta(desAngle)
                ),
                DriveMode.OPEN_LOOP, 
                false, 
                Constants.superstructure.center_of_rotation
            );
        } else {
            swerve.drive(
                new ChassisSpeeds(
                    -throttleSupplier.getAsDouble(),
                    0.0, 
                    0.0
                ),
                DriveMode.OPEN_LOOP, 
                false, 
                Constants.superstructure.center_of_rotation
            );
        }

    }

}
