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
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// import frc.robot.Robot;
import frc.robot.RobotContainer;



public class TeleopDriveCommand extends Command {
  /** Creates a new TeleopDriveCommand. */
  double xSpeed = 0;
  double ySpeed = 0;
  double rot = 0;
  public TeleopDriveCommand() {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(RobotContainer.m_driverController.getLeftY())>0.2){
      xSpeed = -0.7*RobotContainer.m_driverController.getLeftY();
    } else {
      xSpeed = 0;
    }  
    if(Math.abs(RobotContainer.m_driverController.getLeftX())>0.2){
      ySpeed = -0.7*RobotContainer.m_driverController.getLeftX();
    } else {
      ySpeed = 0;
    }  
    if(Math.abs(RobotContainer.m_driverController.getRightX())>0.2){
      rot = -0.7*RobotContainer.m_driverController.getRightX();
    } else {
      rot = 0;
    }  
    RobotContainer.m_DriveSubsystem.drive(xSpeed, ySpeed, rot, false, true);
   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
