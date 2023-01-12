package com.spartronics4915.frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import static com.spartronics4915.frc.Constants.Swerve.*;

public class Trajectories {
    private final Trajectory mTestTrajectory;
    
    public Trajectories() {
        mTestTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, 0)
            ),
            new Pose2d(3, 1, new Rotation2d(Math.PI / 2)),
            new TrajectoryConfig(kMaxSpeed, kMaxAcceleration)
        );
    }

    public Trajectory getTestTrajectory() {
        return mTestTrajectory;
    }
}
