// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ShooterSubsystem);
    addRequirements(RobotContainer.m_IntakeSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

// GO TO SHOOTER COMMAND, THIS DOESN'T WORK


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   // THIS IS FOR XBOX CONTROLLER


     if (RobotContainer.xboxController.getRightBumper()) {

    //Spins intake wheels and fly wheels (loads note)

    IntakeSubsystem.intakeMotor.set(0.5);
    ShooterSubsystem.shooterWheels.set(-0.3);
     }
    /* 
    else if (RobotContainer.xboxController.getLeftBumper()){ 

      // Reverses intake wheels and fly wheels (unloads note)

    IntakeSubsystem.intakeMotor.set(-0.5);
    ShooterSubsystem.shooterWheels.set(0.3);
    }
*/
    else{

      // Stops intake and flywheels

    IntakeSubsystem.intakeMotor.set(0);
    ShooterSubsystem.shooterWheels.set(0);
    }
    

    /* 
    // Intakes (works better when intake motors are faster than flywheels)
    if (RobotContainer.joystick.getRawButton(11)) {
      IntakeSubsystem.intakeMotor.set(0.5);
      ShooterSubsystem.shooterWheels.set(-0.2);
    }

    // Outtakes
    else if (RobotContainer.joystick.getRawButton(10)) {
      IntakeSubsystem.intakeMotor.set(-0.5);
      ShooterSubsystem.shooterWheels.set(0.2);
    }
    else {
      IntakeSubsystem.intakeMotor.set(0);
      ShooterSubsystem.shooterWheels.set(0);
    } */
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
