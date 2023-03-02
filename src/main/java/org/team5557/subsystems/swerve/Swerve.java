package org.team5557.subsystems.swerve;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.state.vision.util.VisionUpdate;

import static org.team5557.subsystems.swerve.util.SwerveSubsystemConstants.*;

import java.util.Map;

import org.library.team2910.math.MathUtils;
import org.library.team4481.SecondOrderSwerveModuleStates;
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
    private double[] moduleTurnSpeeds = new double[4];
    private SecondOrderSwerveModuleStates desiredStates2 = new SecondOrderSwerveModuleStates(desiredStates, moduleTurnSpeeds);


    private SwerveModuleState[] measuredStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState()};
    private SwerveModuleState[] filteredStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState()};
    private SwerveModulePosition[] measuredPositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private SwerveModulePosition[] filteredPositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};


    private DriveMode driveMode = DriveMode.OPEN_LOOP;
    private ChassisSpeeds measuredVelocity = new ChassisSpeeds();
    private ChassisSpeeds filteredVelocity = this.measuredVelocity;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private final GenericEntry motorOutputPercentageLimiterEntry;
    private double motorOutputLimiter;
    private double characterizationVolts = 0.0;
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
        skidVelocityDifference = new TunableNumber("SkidVelocityDifference", 0.75);

        if (DEBUGGING) {
            tab.add(SUBSYSTEM_NAME, this);
            tab.addNumber("vxm", () -> MathUtils.truncate(this.getMeasuredVelocity().vxMetersPerSecond, 2));
            tab.addNumber("vym", () -> MathUtils.truncate(this.getMeasuredVelocity().vyMetersPerSecond, 2));
            tab.addNumber("vxf", () -> MathUtils.truncate(this.getFilteredVelocity().vxMetersPerSecond, 2));
            tab.addNumber("vyf", () -> MathUtils.truncate(this.getFilteredVelocity().vyMetersPerSecond, 2));
            tab.addNumber("Pose Est X", () -> MathUtils.truncate(getPose().getX(), 2));
            tab.addNumber("Pose Est Y", () -> MathUtils.truncate(getPose().getY(), 2));
            tab.addNumber("Pose Est Theta", () -> MathUtils.truncate(getPose().getRotation().getDegrees(), 2));
        }
        
        if (TESTING) {
            tab.add("X - Out", new InstantCommand(() -> setDriveMode(DriveMode.X_OUT)));
            tab.add("Open Loop", new InstantCommand(() -> setDriveMode(DriveMode.OPEN_LOOP)));
            tab.add("Closed Loop", new InstantCommand(() -> setDriveMode(DriveMode.CLOSED_LOOP)));
        }
    }

    @Override
    public void periodic() {
        motorOutputLimiter = motorOutputPercentageLimiterEntry.getDouble(0.0) / 100;
        // update and log gyro inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);

        // update and log the swerve moudles inputs
        //this.desiredStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        this.desiredStates2 = SECOND_KINEMATICS.toSwerveModuleState(chassisSpeeds, Rotation2d.fromDegrees(0.0));
        this.desiredStates = desiredStates2.getSwerveModuleStates();
        this.moduleTurnSpeeds = desiredStates2.getModuleTurnSpeeds();

        SwerveModuleState[] mStates = new SwerveModuleState[4];
        SwerveModulePosition[] mPositions = new SwerveModulePosition[4];
        SwerveModuleState[] fStates = new SwerveModuleState[4];
        SwerveModulePosition[] fPositions = new SwerveModulePosition[4];

        for (SwerveModule swerveModule : swerveModules) {
            int moduleID = swerveModule.getModuleNumber();
            swerveModule.updateAndProcessInputs();
            mStates[moduleID] = swerveModule.getState();
            mPositions[moduleID] = swerveModule.getPosition();

            if(Math.abs(measuredStates[moduleID].speedMetersPerSecond - desiredStates[moduleID].speedMetersPerSecond) > skidVelocityDifference.get()) {
                fStates[moduleID] = new SwerveModuleState(0.0, mStates[moduleID].angle);
                fPositions[moduleID] = new SwerveModulePosition(this.measuredPositions[moduleID].distanceMeters, mStates[moduleID].angle);
            } else {
                fStates[moduleID] = swerveModule.getState();
                fPositions[moduleID] = swerveModule.getPosition();
            }
        }

        this.measuredStates = mStates;
        this.measuredPositions = mPositions;
        this.filteredPositions = fPositions;
        this.filteredStates = fStates;
        
        this.measuredVelocity = KINEMATICS.toChassisSpeeds(measuredStates);
        this.filteredVelocity = KINEMATICS.toChassisSpeeds(filteredStates);
        RobotContainer.state_supervisor.addSkidMeasurement(this.measuredVelocity, this.filteredVelocity);

        estimator.update(getGyroscopeAzimuth(), filteredPositions);

        switch (driveMode) {
            case OPEN_LOOP:
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
                frontLeftModule.setDesiredState(desiredStates[0], moduleTurnSpeeds[0], true, false);
                frontRightModule.setDesiredState(desiredStates[1], moduleTurnSpeeds[1],true, false);
                backLeftModule.setDesiredState(desiredStates[2], moduleTurnSpeeds[2],true, false);
                backRightModule.setDesiredState(desiredStates[3], moduleTurnSpeeds[3],true, false);
                break;
            case CLOSED_LOOP:
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
                frontLeftModule.setDesiredState(desiredStates[0], moduleTurnSpeeds[0], false, false);
                frontRightModule.setDesiredState(desiredStates[1], moduleTurnSpeeds[1],false, false);
                backLeftModule.setDesiredState(desiredStates[2], moduleTurnSpeeds[2],false, false);
                backRightModule.setDesiredState(desiredStates[3], moduleTurnSpeeds[3],false, false);
                break;
            case X_OUT:
                this.desiredStates = X_OUT_STATES;
                frontLeftModule.setDesiredState(desiredStates[0], true, true);
                frontRightModule.setDesiredState(desiredStates[1], true, true);
                backLeftModule.setDesiredState(desiredStates[2], true, true);
                backRightModule.setDesiredState(desiredStates[3], true, true);
                break;
            case FF_CHARACTERIZATION:
                frontLeftModule.setVoltageForCharacterization(characterizationVolts);
                frontRightModule.setVoltageForCharacterization(characterizationVolts);
                backLeftModule.setVoltageForCharacterization(characterizationVolts);
                backRightModule.setVoltageForCharacterization(characterizationVolts);
                break;
        }

        Logger.getInstance().recordOutput("Swerve/Drive Mode", getDriveMode().toString());

        Logger.getInstance().recordOutput("Swerve/Velocity/Vx", filteredVelocity.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Velocity/Vy", filteredVelocity.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Velocity/V0", filteredVelocity.omegaRadiansPerSecond);

        Logger.getInstance().recordOutput("Swerve/States/Filtered States", filteredStates);
        Logger.getInstance().recordOutput("Swerve/States/Measured States", measuredStates);
    }


    /* GYRO */
    public Rotation2d getGyroscopeAzimuth() {
        return Rotation2d.fromDegrees(gyroInputs.azimuthDeg);
    }

    public double getGyroscopePitch() {
        return gyroInputs.pitchDeg - this.pitchOffset;
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

    public void addVisionMeasurement(VisionUpdate update) {
        estimator.addVisionMeasurement(update.measuredPose, update.timestamp, update.stdDevs);
    }
    
    /* CHASSIS SPEEDS */
    public ChassisSpeeds getFilteredVelocity() {
        return this.filteredVelocity;
    }

    public ChassisSpeeds getMeasuredVelocity() {
        return this.measuredVelocity;
    }

    /* OUTPUT LIMITER */
    public double getMotorOutputLimiter() {
        return motorOutputLimiter;
    }

    /* MODULE STATES/POSITIONS */
    public SwerveModulePosition[] getModulePositions() {
        return this.filteredPositions;
    }

    public SwerveModuleState[] getModuleStates() {
        return this.filteredStates;
    }

    public SwerveModuleState[] getDesiredStates() {
        return this.desiredStates;
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     */
    public void drive(ChassisSpeeds chassisSpeeds, DriveMode driveMode, boolean fieldRelative, Translation2d centerOfRotation) {
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
        this.drive(chassisSpeeds, DriveMode.OPEN_LOOP, true, Constants.superstructure.center_of_rotation);
    }

    public void runCharacterizationVolts(double volts) {
        this.driveMode = DriveMode.FF_CHARACTERIZATION;
        this.characterizationVolts = volts;
    }

    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (SwerveModule swerveModule : swerveModules) {
            driveVelocityAverage += swerveModule.getState().speedMetersPerSecond;
        }
        return driveVelocityAverage / 4.0;
    }

    public enum DriveMode {
        OPEN_LOOP,
        CLOSED_LOOP,
        X_OUT,
        FF_CHARACTERIZATION
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
