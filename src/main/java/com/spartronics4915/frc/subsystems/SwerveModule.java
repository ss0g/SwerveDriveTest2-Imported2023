package com.spartronics4915.frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.spartronics4915.frc.Constants.Swerve.*;

public class SwerveModule {
    private final int mModuleNumber;
    private double mAbsoluteOffset;
    private double mLastAngle;

    private CANSparkMax mDriveMotor;
    private CANSparkMax mAngleMotor;

    private RelativeEncoder mDriveEncoder;
    private RelativeEncoder mIntegratedAngleEncoder;
    private AnalogEncoder mSteeringEncoder;

    private final SparkMaxPIDController mDriveController;
    private final SparkMaxPIDController mAngleController;
    private boolean mInvertDirection;

    private SwerveModuleState mDesiredState;

    private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID, 
                        double absoluteOffset, boolean invertDirection) {
        mModuleNumber = moduleNumber;

        mAbsoluteOffset = absoluteOffset;
        mInvertDirection = invertDirection;
        
        mDriveMotor = kMotorConstructor.apply(driveMotorID);
        mDriveEncoder = mDriveMotor.getEncoder();
        mDriveController = mDriveMotor.getPIDController();
        configureDriveMotor();
        
        mAngleMotor = kMotorConstructor.apply(angleMotorID);
        mIntegratedAngleEncoder = mAngleMotor.getEncoder();
        mAngleController = mAngleMotor.getPIDController();
        configureAngleMotor();

        mSteeringEncoder = new AnalogEncoder(new AnalogInput(encoderID));

        mDesiredState = new SwerveModuleState();

        mLastAngle = getState().angle.getRadians();
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
        this(moduleNumber, constants.driveMotorID, constants.angleMotorID, constants.encoderID, constants.absoluteOffset,
        constants.invertDirection);
    }

    public void forceModuleOrientation(Rotation2d newAngle, boolean isOpenLoop){
        // Forces all of the modules to a desired orientation.  Will not change the speed
        // Mainly for testing, be careful if you use this.

        var currentState = this.getState();
        var newState = new SwerveModuleState(currentState.speedMetersPerSecond, newAngle);

        this.setDesiredState(newState, isOpenLoop);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        // if(mInvertDirection)
        // {
        //     System.out.println("*************INVERTING DIRECTION**************");
        // }
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mDesiredState = desiredState;

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
            mDriveMotor.set(percentOutput);
        } else {
            mDriveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity, 0,
                mFeedforward.calculate(desiredState.speedMetersPerSecond)
            );
        }

        boolean suppressTurningAtLowSpeed = false;
        double angle = (suppressTurningAtLowSpeed && Math.abs(desiredState.speedMetersPerSecond) < kMaxSpeed * 0.01) ?
            mLastAngle :
            desiredState.angle.getRadians();

        mAngleController.setReference(angle, ControlType.kPosition);
        mLastAngle = angle;
    }

    public int getModuleNumber() {
        return mModuleNumber;
    }

    public SwerveModuleState getDesiredState() {
        return mDesiredState;
    }
    public void putSmartDashboardValues() {
        // SmartDashboard.putNumber("mod " + mModuleNumber + " encoder", mSteeringEncoder.getDistance());
        
		// SmartDashboard.putNumber("mod " + mModuleNumber + " absEnc.getDistance()", mSteeringEncoder.getDistance() - mAngleOffset);
		
		SmartDashboard.putNumber("mod " + mModuleNumber + " encoder absolute", mSteeringEncoder.getAbsolutePosition());
		SmartDashboard.putNumber("mod " + mModuleNumber + " encoder relative", mIntegratedAngleEncoder.getPosition());

        // SmartDashboard.putNumber("mod " + mModuleNumber + " integrated", mIntegratedAngleEncoder.getPosition());
        // SmartDashboard.putNumber("mod " + mModuleNumber + " velocity", mDriveEncoder.getVelocity());

        // SmartDashboard.putNumber("mod " + mModuleNumber + " desired angle", mDesiredState.angle.getRadians());
    }

    public void resetToAbsolute() {
        Rotation2d encoderAngle = getShiftedAbsoluteEncoderRotation();
        if (mInvertDirection) {

            encoderAngle = encoderAngle.rotateBy(Rotation2d.fromDegrees(180));
        }
        System.out.println(mModuleNumber + " " + getAbsoluteEncoderValue() + " " + encoderAngle.getRadians());
        mIntegratedAngleEncoder.setPosition(encoderAngle.getRadians());
        mDriveController.setReference(encoderAngle.getRadians(), ControlType.kPosition);
    }

    public double getAbsoluteEncoderValue() {
        return 1.0 - mSteeringEncoder.getAbsolutePosition();
    }

    public Rotation2d getShiftedAbsoluteEncoderRotation() {
        return Rotation2d.fromRotations(getAbsoluteEncoderValue()).minus(
            Rotation2d.fromRotations(mAbsoluteOffset));
    }

    public double getShiftedAbsoluteEncoderRotations() {
        return getShiftedAbsoluteEncoderRotation().getRotations();
    }

    public double getRelativeEncoderValue() {
        return mIntegratedAngleEncoder.getPosition();
    }

    private void configureDriveMotor() {
        mDriveMotor.restoreFactoryDefaults(); // ?
        mDriveMotor.setSmartCurrentLimit(Drive.kContinuousCurrentLimit);
        mDriveMotor.setInverted(kDriveMotorsAreInverted);
        mDriveMotor.setIdleMode(kDriveIdleMode);
        mDriveEncoder.setVelocityConversionFactor(Drive.kVelocityConversionFactor);
        mDriveEncoder.setPositionConversionFactor(Drive.kPositionConversionFactor);
        mDriveController.setP(Drive.kP);
        mDriveController.setI(Drive.kI);
        mDriveController.setD(Drive.kD);
        mDriveController.setFF(Drive.kFF);
        mDriveMotor.enableVoltageCompensation(kVoltageCompensation);
        mDriveMotor.burnFlash();
        mDriveEncoder.setPosition(0.0);
    }
    
    private void configureAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Angle.kContinuousCurrentLimit);
        mAngleMotor.setIdleMode(kAngleIdleMode);
        mIntegratedAngleEncoder.setPositionConversionFactor(Angle.kPositionConversionFactor);
        mAngleController.setP(Angle.kP);
        mAngleController.setI(Angle.kI);
        mAngleController.setD(Angle.kD);
        mAngleController.setFF(Angle.kFF);
        mAngleMotor.enableVoltageCompensation(kVoltageCompensation);
        mAngleMotor.burnFlash();
    }

    public SwerveModuleState getState() {
        double velocity = mDriveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromRadians(mIntegratedAngleEncoder.getPosition()); // TODO: why isnt this using the analog encoder
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double drivePosition = mDriveEncoder.getPosition();
        Rotation2d angle = getState().angle;
        return new SwerveModulePosition(drivePosition, angle);
    }

    public void zeroPIDP(){
        mAngleController.setP(0);
    }
}
