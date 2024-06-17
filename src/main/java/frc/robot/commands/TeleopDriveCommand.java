// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import frc.robot.Robot;
import frc.robot.RobotContainer;



public class TeleopDriveCommand extends Command {
  /** Creates a new TeleopDriveCommand. */
  double xSpeed = 0;
  double ySpeed = 0;
  double rot = 0;
  double SpeedMultiplier;

  private boolean isRobotOriented;

  public TeleopDriveCommand() {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_ShooterSubsystem.servo.set(1);
    RobotContainer.m_ShooterSubsystem.Stopper.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.xboxController.getPOV() == 270 || RobotContainer.xboxController.getPOV() == 90) {
    isRobotOriented = true;
    }
    else {
    isRobotOriented = false;
    }

    
    if (isRobotOriented == false) {
    SpeedMultiplier = 1-(0.85*RobotContainer.m_driverController.getLeftTriggerAxis());
    if(Math.abs(RobotContainer.m_driverController.getLeftY())>0.2){
      xSpeed = -SpeedMultiplier*RobotContainer.m_driverController.getLeftY();
    } else {
      xSpeed = 0;
    }  
    if(Math.abs(RobotContainer.m_driverController.getLeftX())>0.2){
      ySpeed = -SpeedMultiplier*RobotContainer.m_driverController.getLeftX();
    } else {
      ySpeed = 0;
    }  
    if(Math.abs(RobotContainer.m_driverController.getRightX())>0.2){
      rot = -RobotContainer.m_driverController.getRightX();
    } else {
      rot = 0;
    }
    

    RobotContainer.m_DriveSubsystem.drive(xSpeed, ySpeed, rot, true, true);
  }
  else if (RobotContainer.xboxController.getPOV() == 270) {
    RobotContainer.m_DriveSubsystem.drive(0,0.3,rot,false,true);
  }
  else if (RobotContainer.xboxController.getPOV() == 90) {
    RobotContainer.m_DriveSubsystem.drive(0,-0.3,rot,false,true);
  }
   
    SmartDashboard.putNumber("Motor Power X", xSpeed);
    SmartDashboard.putNumber("Motor Power Y", ySpeed);
    SmartDashboard.putNumber("Among Us", RobotContainer.xboxController.getPOV());
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
