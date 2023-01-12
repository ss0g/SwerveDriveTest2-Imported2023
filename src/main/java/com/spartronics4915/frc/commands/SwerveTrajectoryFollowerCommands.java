package com.spartronics4915.frc.commands;

import java.util.ArrayList;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.spartronics4915.frc.subsystems.Swerve;
import com.spartronics4915.frc.subsystems.SwervePoseEstimator;

public class SwerveTrajectoryFollowerCommands {
	private final Swerve mSwerve;
	private final SwervePoseEstimator mPoseEstimator;
	private final SwerveDriveKinematics mKinematics;
	private final PIDController mXPID, mYPID;
	private final ProfiledPIDController mThetaPID;

	public SwerveTrajectoryFollowerCommands(
		Swerve swerve,
		SwervePoseEstimator poseEstimator,
		double xP, double yP, double thetaP
	) {
		mPoseEstimator = poseEstimator;
		mSwerve = swerve;

		mKinematics = new SwerveDriveKinematics();
		mXPID = new PIDController(xP, 0, 0);
		mYPID = new PIDController(yP, 0, 0);
		mThetaPID = new ProfiledPIDController(
			thetaP, 0, 0, new TrapezoidProfile.Constraints(0, 0)
		);
		mThetaPID.enableContinuousInput(0, 2 * Math.PI);
	}

	private ProfiledPIDController updateThetaPID(
		double maxAngularVelocity, double maxAngularAcceleration // radians per second
	) {
		mThetaPID.setConstraints(new TrapezoidProfile.Constraints(
			maxAngularVelocity, maxAngularAcceleration
		));
		return mThetaPID;
	}

	class FollowTrajectory extends SwerveControllerCommand {
		public FollowTrajectory(
			ArrayList<Pose2d> waypoints, // meters
			double startVelocity, double endVelocity, double maxVelocity, // meters per second
			double maxAngularVelocity, double maxAngularAcceleration, // radians per second
			double maxAccel // meters per second squared
		) {
			super(
				TrajectoryGenerator.generateTrajectory(
					waypoints,
					new TrajectoryConfig(maxVelocity, maxAccel)
						.setStartVelocity(startVelocity)
						.setEndVelocity(endVelocity)
				),
				mPoseEstimator::getPose,
				mKinematics,
				mXPID, mYPID,
				updateThetaPID(
					maxAngularVelocity,
					maxAngularAcceleration
				),
				mSwerve::setModuleStates,
				mSwerve
			);

		}
	}
}
