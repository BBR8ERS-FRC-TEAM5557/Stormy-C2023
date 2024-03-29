package org.team5557.subsystems.swerve.util;

import java.util.function.DoubleSupplier;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.swerve.Swerve;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RawControllers extends SubsystemBase {
    public final ProfiledPIDController thetaController;
    public final PIDController alignController;

    public final PPHolonomicDriveController follower;
    public final PIDController rotationController;
    public final PIDController xController;
    public final PIDController yController;
    public Rotation2d startAngle;
    private final Swerve swerve;

    /** Creates a new Trajectory. */
    public RawControllers() {
        thetaController = new ProfiledPIDController(
            Constants.follower.theta_kP, 
            Constants.follower.theta_kI, 
            Constants.follower.theta_kD, 
            new Constraints(Constants.follower.theta_kV, Constants.follower.theta_kA)
        );
        // Setup thetaController used for auton and automatic turns
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(1.0));

        alignController = new PIDController(Constants.follower.auto_kP, Constants.follower.auto_kI, Constants.follower.auto_kD);
        alignController.enableContinuousInput(-Math.PI, Math.PI);

        rotationController = new PIDController(Constants.follower.auto_kP, Constants.follower.auto_kI, Constants.follower.auto_kD);
        xController = new PIDController(Constants.follower.translation_kP, Constants.follower.translation_kI, Constants.follower.translation_kD);
        yController = new PIDController(Constants.follower.translation_kP, Constants.follower.translation_kI, Constants.follower.translation_kD);

        follower = new PPHolonomicDriveController(xController, yController, rotationController);
        follower.setTolerance(Constants.follower.tolerance);
        
        this.swerve = RobotContainer.swerve;
    }

    public void resetTheta() {
        startAngle = swerve.getPose().getRotation();
        thetaController.reset(startAngle.getRadians(), swerve.getFilteredVelocity().omegaRadiansPerSecond);
    }

    public Rotation2d getStartAngle() {
        return startAngle;
    }

    public double calculateTheta(double goalAngleRadians) {
        return thetaController.calculate(swerve.getPose().getRotation().getRadians(), goalAngleRadians);
    }

    public DoubleSupplier calculateThetaSupplier(DoubleSupplier goalAngleSupplierRadians) {
        return () -> calculateTheta(goalAngleSupplierRadians.getAsDouble());
    }

    public DoubleSupplier calculateThetaSupplier(double goalAngle) {
        return calculateThetaSupplier(() -> goalAngle);
    }

    //FOR NON PROFILED ANGLE STUFF
    public void resetAlign() {
        startAngle = swerve.getPose().getRotation();
        thetaController.reset(startAngle.getRadians(), swerve.getFilteredVelocity().omegaRadiansPerSecond);
    }

    public double calculateAlign(double goalAngleRadians) {
        return alignController.calculate(swerve.getPose().getRotation().getRadians(), goalAngleRadians);
    }

    public DoubleSupplier calculateAlignSupplier(DoubleSupplier goalAngleSupplierRadians) {
        return () -> calculateAlign(goalAngleSupplierRadians.getAsDouble());
    }

    public DoubleSupplier calculateAlignSupplier(double goalAngle) {
        return calculateThetaSupplier(() -> goalAngle);
    }

    @Override
    public void periodic() {

    }

    public void resetTranslation() {
        xController.reset();
        yController.reset();
    }
}