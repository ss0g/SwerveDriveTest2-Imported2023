package com.spartronics4915.frc.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc.Constants.Swerve.*;
import static com.spartronics4915.frc.Constants.Camera.*;

import java.util.Arrays;

public class Swerve extends SubsystemBase {
    // private SwerveDriveOdometry mOdometry;
    private SwerveDrivePoseEstimator mPoseEstimator;

    private SwerveModule[] mModules;

    private AHRS mNavX;
    private PhotonCamera mFrontCamera;

    private boolean mIsFieldRelative = true;

    public Swerve() {
        mNavX = new AHRS();
        mNavX.reset();

        mFrontCamera = new PhotonCamera(NetworkTableInstance.getDefault(), kFrontCameraName);

        mModules = new SwerveModule[] {
            new SwerveModule(0, Module0.kConstants),
            new SwerveModule(1, Module1.kConstants),
            new SwerveModule(2, Module2.kConstants),
            new SwerveModule(3, Module3.kConstants)
        };

        // mOdometry = new SwerveDriveOdometry(kKinematics, getYaw(), getPositions());
        mPoseEstimator = new SwerveDrivePoseEstimator(
            kKinematics,
            getYaw(),
            getPositions(),
            new Pose2d(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.9, 0.9, 0.9)
        ); // TODO: put a different pose in here
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds;
        SmartDashboard.putBoolean("field relative", mIsFieldRelative);
        if (mIsFieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            );
        }

        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(moduleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public void setFieldRelative(boolean fieldRelative) {
        mIsFieldRelative = fieldRelative;
    }

    public void toggleFieldRelative() {
        mIsFieldRelative = !mIsFieldRelative;
    }

    public boolean getFieldRelative() {
        return mIsFieldRelative;
    }

    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-mNavX.getYaw());
    }

    public void resetOdometry(Pose2d pose) {
        mPoseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetModuleZeroes() {
        for (SwerveModule mod : mModules) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroNavX() {
        mNavX.reset();
    }

    public void zeroModules() {
        SwerveModuleState[] zeroedStates = new SwerveModuleState[4];
        Arrays.fill(zeroedStates, new SwerveModuleState(0, new Rotation2d(0)));
        // for (SwerveModuleState state : zeroedStates) {
        //     state = new SwerveModuleState(0, new Rotation2d(0));
        // }
        setModuleStates(zeroedStates);
    }

    @Override
    public void periodic() {
        var frontLatestResult = mFrontCamera.getLatestResult();
        if (frontLatestResult.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - frontLatestResult.getLatencyMillis();
            var bestTarget = frontLatestResult.getBestTarget();
            int bestTargetID = bestTarget.getFiducialId();
            Transform3d camToTargetTrans = bestTarget.getBestCameraToTarget();
            Transform2d camToTargetTrans2d = new Transform2d(
                camToTargetTrans.getTranslation().toTranslation2d(),
                camToTargetTrans.getRotation().toRotation2d()
            );
            Pose2d camPose = kTagPoses[bestTargetID].transformBy(camToTargetTrans2d.inverse());
            mPoseEstimator.addVisionMeasurement(camPose.transformBy(kFrontCameraToRobot), imageCaptureTime);
        }
        mPoseEstimator.update(getYaw(), getPositions());
        for (SwerveModule mod : mModules) {
            mod.putSmartDashboardValues();
        }
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rotation degrees", getPose().getRotation().getDegrees());
    }
}
