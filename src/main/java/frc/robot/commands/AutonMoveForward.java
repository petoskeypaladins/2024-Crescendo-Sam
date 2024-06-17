// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonMoveForward extends SequentialCommandGroup {
  /** Creates a new AutonMoveForward2. */
  public AutonMoveForward() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared) //
      .setKinematics(DriveConstants.kDriveKinematics); 

    edu.wpi.first.math.trajectory.Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), //
        List.of(//
          new Translation2d(0,1)
      ),
      new Pose2d(0,1, Rotation2d.fromDegrees(0)),
      trajectoryConfig
      );

    PIDController xController = new PIDController(AutoConstants.kPYController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
         trajectory, 
         RobotContainer.m_DriveSubsystem::getPose,
         DriveConstants.kDriveKinematics,
         xController,
         yController,
         thetaController,
         RobotContainer.m_DriveSubsystem::setModuleStates,
         RobotContainer.m_DriveSubsystem);

    addCommands(
      new InstantCommand(() -> RobotContainer.m_DriveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand
    );
  }
}
