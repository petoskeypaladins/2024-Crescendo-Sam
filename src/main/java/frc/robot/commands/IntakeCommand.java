// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ShooterSubsystem);
    addRequirements(RobotContainer.m_IntakeSubsytem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_IntakeSubsytem.intake();
    RobotContainer.m_ShooterSubsystem.shooterIntake();
  }

// GO TO SHOOTER COMMAND, THIS DOESN'T WORK


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_IntakeSubsytem.stop();
    RobotContainer.m_ShooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
