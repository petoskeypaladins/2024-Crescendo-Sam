// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    RobotContainer.m_DriveSubsystem.drive(xSpeed, ySpeed, rot, true, true);
    
    // SmartDashboard.putNumber("Controller X", RobotContainer.m_driverController.getRightX());
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
