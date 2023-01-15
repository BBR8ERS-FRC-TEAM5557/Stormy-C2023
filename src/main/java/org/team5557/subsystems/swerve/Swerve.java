package org.team5557.subsystems.swerve;

import org.team5557.Constants;
import static org.team5557.subsystems.swerve.SwerveSubsystemConstants.*;

import java.util.Map;

import org.library.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;
import org.team5557.subsystems.gyro.GyroIO;
import org.team5557.subsystems.gyro.GyroIOInputsAutoLogged;
import org.team5557.subsystems.gyro.GyroIOPigeon2;
import org.team5557.subsystems.swerve.module.SwerveModule;
import org.team5557.subsystems.swerve.module.SwerveModuleIOSparkMax;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Swerve extends SubsystemBase {
    private static final String SUBSYSTEM_NAME = Constants.shuffleboard.swerve_readout_key;
    private static final boolean TESTING = true;
    private static final boolean DEBUGGING = true;

    private final SwerveDrivePoseEstimator estimator;
    private final GyroIO gyroIO = new GyroIOPigeon2(Constants.ports.pigeon);
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private double pitchOffset = 0.0;

    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private SwerveModuleState[] desiredStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState()};
    
    private SwerveModuleState[] currentStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState()};
    private SwerveModuleState[] lastStates = this.currentStates;
    private SwerveModulePosition[] currentPositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private SwerveModulePosition[] lastPositions = this.currentPositions;


    private DriveMode driveMode = DriveMode.OPEN_LOOP;
    private ChassisSpeeds currentVelocity = new ChassisSpeeds();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private Translation2d centerOfRotation = new Translation2d();

    private final GenericEntry motorOutputPercentageLimiterEntry;
    private double motorOutputLimiter;
    private final TunableNumber skidVelocityDifference;
    
    public Swerve() {
        frontLeftModule = new SwerveModule(
            new SwerveModuleIOSparkMax(
                0, 
                FL_DRIVE_MOTOR, 
                FL_STEER_MOTOR, 
                FL_CANCODER, 
                FL_OFFSET), 
            0, 
            MAX_VELOCITY_METERS_PER_SECOND
        );

        frontRightModule = new SwerveModule(
            new SwerveModuleIOSparkMax(
                1, 
                FR_DRIVE_MOTOR, 
                FR_STEER_MOTOR, 
                FR_CANCODER, 
                FR_OFFSET), 
            1, 
            MAX_VELOCITY_METERS_PER_SECOND
        );

        backLeftModule = new SwerveModule(
            new SwerveModuleIOSparkMax(
                2, 
                BL_DRIVE_MOTOR, 
                BL_STEER_MOTOR, 
                BL_CANCODER, 
                BL_OFFSET), 
            2, 
            MAX_VELOCITY_METERS_PER_SECOND
        );

        backRightModule = new SwerveModule(
            new SwerveModuleIOSparkMax(
                3, 
                BR_DRIVE_MOTOR, 
                BR_STEER_MOTOR, 
                BR_CANCODER, 
                BR_OFFSET), 
            3, 
            MAX_VELOCITY_METERS_PER_SECOND
        );

        swerveModules[0] = frontLeftModule;
        swerveModules[1] = frontRightModule;
        swerveModules[2] = backLeftModule;
        swerveModules[3] = backRightModule;

        estimator = new SwerveDrivePoseEstimator(
            KINEMATICS, 
            getGyroscopeAzimuth(), 
            DEFAULT_POSITIONS, 
            new Pose2d(),
            Constants.estimator.stateStdDevs, // estimator values (x, y, rotation) std-devs
            Constants.estimator.normalVisionStdDevs); // Vision (x, y, rotation) std-devs*/
            
        //potentially takeout so that we are super stable on the charging station??
        new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
            frontLeftModule.setDriveBrakeMode(true);
            frontLeftModule.setAngleBrakeMode(true);
            frontRightModule.setDriveBrakeMode(true);
            frontRightModule.setAngleBrakeMode(true);
            backLeftModule.setDriveBrakeMode(true);
            backLeftModule.setAngleBrakeMode(true);
            backRightModule.setDriveBrakeMode(true);
            backRightModule.setAngleBrakeMode(true);
        }, () -> {
            frontLeftModule.setDriveBrakeMode(false);
            frontLeftModule.setAngleBrakeMode(false);
            frontRightModule.setDriveBrakeMode(false);
            frontRightModule.setAngleBrakeMode(false);
            backLeftModule.setDriveBrakeMode(false);
            backLeftModule.setAngleBrakeMode(false);
            backRightModule.setDriveBrakeMode(false);
            backRightModule.setAngleBrakeMode(false);
            }));


        ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
        motorOutputPercentageLimiterEntry = tab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 100.0, "Block increment", 10.0)).withPosition(0, 3)
            .getEntry();
        skidVelocityDifference = new TunableNumber("Swerve/SkidVelocityDifference", 1.0);

        if (DEBUGGING) {
            tab.add(SUBSYSTEM_NAME, this);
            tab.addNumber("vx", () -> this.getCurrentVelocity().vxMetersPerSecond);
            tab.addNumber("vy", () -> this.getCurrentVelocity().vyMetersPerSecond);
            tab.addNumber("Pose Est X", () -> getPose().getX());
            tab.addNumber("Pose Est Y", () -> getPose().getY());
            tab.addNumber("Pose Est Theta", () -> getPose().getRotation().getDegrees());
        }
        
        if (TESTING) {
            tab.add("X - Out", new InstantCommand(() -> setDriveMode(DriveMode.X_OUT)));
            tab.add("Open Loop", new InstantCommand(() -> setDriveMode(DriveMode.OPEN_LOOP)));
            tab.add("Closed Loop", new InstantCommand(() -> setDriveMode(DriveMode.CLOSED_LOOP)));
        }
    }

    @Override
    public void periodic() {
        // update and log gyro inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);

        // update and log the swerve moudles inputs
        this.desiredStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        for (SwerveModule swerveModule : swerveModules) {
            int moduleID = swerveModule.getModuleNumber();
            swerveModule.updateAndProcessInputs();
            this.currentStates[moduleID] = swerveModule.getState();
            this.currentPositions[moduleID] = swerveModule.getPosition();
        }
        

        motorOutputLimiter = motorOutputPercentageLimiterEntry.getDouble(0.0) / 100;
    
        /*
        //ORBIT 1690 SKID Detection implementation...
        for(int i = 0; i < 4; i++) {
            if(Math.abs(currentStates[i].speedMetersPerSecond - desiredStates[i].speedMetersPerSecond) > skidVelocityDifference.get()) {//SKID_VELOCITY_DIFFERENCE) {
                currentStates[i].speedMetersPerSecond = 0.0;
                currentPositions[i].distanceMeters = lastPositions[i].distanceMeters;
            }
        }*/
        this.lastStates = currentStates;
        this.lastPositions = currentPositions;
        this.currentVelocity = KINEMATICS.toChassisSpeeds(currentStates);

        estimator.update(getGyroscopeAzimuth(), currentPositions);

        switch (driveMode) {
            case OPEN_LOOP:
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
                //frontLeftModule.setDesiredState(desiredStates[0], true, false);
                frontRightModule.setDesiredState(desiredStates[1], true, false);
                //System.out.println(desiredStates[1].toString());
                //backLeftModule.setDesiredState(desiredStates[2], true, false);
                //backRightModule.setDesiredState(desiredStates[3], true, false);
            case CLOSED_LOOP:
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
                frontLeftModule.setDesiredState(desiredStates[0], false, false);
                frontRightModule.setDesiredState(desiredStates[1], false, false);
                backLeftModule.setDesiredState(desiredStates[2], false, false);
                backRightModule.setDesiredState(desiredStates[3], false, false);
            case X_OUT:
                this.desiredStates = X_OUT_STATES;
                frontLeftModule.setDesiredState(desiredStates[0], true, true);
                frontRightModule.setDesiredState(desiredStates[1], true, true);
                backLeftModule.setDesiredState(desiredStates[2], true, true);
                backRightModule.setDesiredState(desiredStates[3], true, true);
        }

        Logger.getInstance().recordOutput("Swerve/Estimator/Pose", estimator.getEstimatedPosition());
        Logger.getInstance().recordOutput("Swerve/Drive Mode", getDriveMode().toString());
        Logger.getInstance().recordOutput("Swerve/Vx", currentVelocity.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Vy", currentVelocity.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/V0", currentVelocity.omegaRadiansPerSecond);
    }


    /* GYRO */
    public Rotation2d getGyroscopeAzimuth() {
        return Rotation2d.fromDegrees(gyroInputs.azimuthDeg);
    }

    public Rotation2d getGyroscopePitch() {
        return Rotation2d.fromDegrees(gyroInputs.pitchDeg - this.pitchOffset);
    }

    public double getGyroscopePitchVelocity() {
        return gyroInputs.pitchVelocityDegPerSec;
    }

    public void zeroGyroscopePitch() {
        this.pitchOffset = gyroInputs.pitchDeg;
    }

    /* POSES AND ESTIMATOR */
    public SwerveDrivePoseEstimator getEstimator() {
        return estimator;
    }

    /**
     * Returns the position of the robot
     */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void setPose(Pose2d pose) {
        estimator.resetPosition(getGyroscopeAzimuth(), getModulePositions(), pose);
    }

    
    /* CHASSIS SPEEDS */
    public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
    }

    /* OUTPUT LIMITER */
    public double getMotorOutputLimiter() {
        return motorOutputLimiter;
    }

    /* MODULE STATES/POSITIONS */
    public SwerveModulePosition[] getModulePositions() {
        return currentPositions;
    }

    public SwerveModuleState[] getModuleStates() {
        return currentStates;
    }

    public SwerveModuleState[] getDesiredStates() {
        return desiredStates;
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     */
    public void drive(ChassisSpeeds chassisSpeeds, DriveMode driveMode, boolean fieldRelative, Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
        if (fieldRelative) {
            this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getPose().getRotation());
        } else {
            this.chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        }
        if(driveMode != getDriveMode()) {
            setDriveMode(driveMode);
        }
    }

    /*
     * Chassis speeds consumer for PathPlanner
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.drive(chassisSpeeds, DriveMode.CLOSED_LOOP, true, Constants.superstructure.center_of_rotation);
    }

    public enum DriveMode {
        OPEN_LOOP,
        CLOSED_LOOP,
        X_OUT
    }

    /*
     * Set DriveMode for swerveeee
     * @param DriveMode - mode to set to
     */
    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    /*
     * Get DriveMode for swerveeee
     * @param DriveMode - mode to set to
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }
}
