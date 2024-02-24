// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutonomousSubsystem extends SubsystemBase {
  /** Creates a new AutonomousSubsystem. */
  public AutonomousSubsystem(edu.wpi.first.math.trajectory.Trajectory trajectory) {

TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared) //
      .setKinematics(DriveConstants.kDriveKinematics);

     trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), //
        List.of(//
          new Translation2d(0, 0.5),
          new Translation2d(1,1),
          new Translation2d(1,0),
          new Translation2d(0.5,-0.5),
          new Translation2d(0,-0.5)
      ),
      new Pose2d(0,0, Rotation2d.fromDegrees(270)),
      trajectoryConfig //
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
