// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ClimbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //right climb
    if (RobotContainer.joystick.getRawButton(6)) {
      RobotContainer.m_ClimbSubsystem.rightClimb.set(0.85);
    }
    else if (RobotContainer.joystick.getRawButton(4)) {
      RobotContainer.m_ClimbSubsystem.rightClimb.set(-0.85);
    }
    else {
      RobotContainer.m_ClimbSubsystem.rightClimb.set(0);
    }
    //left climb
  /*   if (RobotContainer.joystick.getRawButton(3)) {
      RobotContainer.m_ClimbSubsystem.leftClimb.set(-0.5);
    }
    else if (RobotContainer.joystick.getRawButton(5)) {
      RobotContainer.m_ClimbSubsystem.leftClimb.set(0.5);
    }
    else {
       RobotContainer.m_ClimbSubsystem.leftClimb.set(0);
    }*/
     if (RobotContainer.joystick.getRawButton(5)) {
      RobotContainer.m_ClimbSubsystem.leftClimb.set(-0.85);
    }
    else if (RobotContainer.joystick.getRawButton(3)) {
      RobotContainer.m_ClimbSubsystem.leftClimb.set(0.85);
    }
    else {
      RobotContainer.m_ClimbSubsystem.leftClimb.set(0);
    }
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
