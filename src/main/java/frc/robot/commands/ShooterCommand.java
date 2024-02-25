// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_IntakeSubsytem);
    addRequirements(RobotContainer.m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.servo.setPosition(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

       // THIS IS FOR XBOX CONTROLLER
      
/* 
      //Spins intake wheels and flywheels (loads note)
      if (RobotContainer.xboxController.getRightBumper()) {
        IntakeSubsystem.intakeMotor.set(0.5);
        ShooterSubsystem.shooterWheels.set(0.3);
      }
      // Reverses intake wheels and flywheels (unloads note)
      else if (RobotContainer.xboxController.getLeftBumper()){ 
        IntakeSubsystem.intakeMotor.set(-0.5);
        ShooterSubsystem.shooterWheels.set(-0.3);
      } */

      // Revs intake wheels to shoot
      if (RobotContainer.joystick.getRawButton(1)) {
      ShooterSubsystem.shooterWheels.set(0.8);
    
    }
      else if (RobotContainer.joystick.getRawButton(11)) {
        ShooterSubsystem.shooterWheels.set(0.21);
      }
      // Stops intake and flywheels
      else {
        IntakeSubsystem.intakeMotor.set(0);
        ShooterSubsystem.shooterWheels.set(0);
      }
      
          
    // Pivots Up (Pull stick back) (negative value)
    if (RobotContainer.joystick.getRawAxis(1) > 0.02) {
      ShooterSubsystem.pivotMotor.set(-1 * RobotContainer.joystick.getRawAxis(1));
    }
    // Pivots Down (Push stick forward) (positive value)
    else if (RobotContainer.joystick.getRawAxis(1) < -0.02) {
      ShooterSubsystem.pivotMotor.set(-1 * RobotContainer.joystick.getRawAxis(1));
    }
     //preset motor in shooting POSITION. MAX VALUE IS 0.332, min value is 0.326!
    else if (RobotContainer.joystick.getRawButton(7)) {
      if(ShooterSubsystem.pivotEncoder.getAbsolutePosition() < 0.326) {
        ShooterSubsystem.pivotMotor.set(-0.2 - (0.326 - ShooterSubsystem.pivotEncoder.getAbsolutePosition()));
      }
      else if (ShooterSubsystem.pivotEncoder.getAbsolutePosition() > 0.332) {
        ShooterSubsystem.pivotMotor.set(0.2 + (ShooterSubsystem.pivotEncoder.getAbsolutePosition() - 0.332));
      }
      else {
        ShooterSubsystem.pivotMotor.set(0);
      }
    } 
    else {
      ShooterSubsystem.pivotMotor.set(0);
    }

    /* 
    // Revs shooter wheels
    if (RobotContainer.joystick.getRawButton(1)) {
      ShooterSubsystem.shooterWheels.set(0.8);
    }
    else {
      ShooterSubsystem.shooterWheels.set(0);
    }
*/
    // Moves servo to shoot
    if (RobotContainer.joystick.getRawButton(2)) {
      ShooterSubsystem.servo.setPosition(0);
    }
    // Moves servo back
    else {
      ShooterSubsystem.servo.setPosition(1);
    }
    

/* 
// Intake shenanigans
// Intakes (works better when intake motors are faster than flywheels)
    if (RobotContainer.joystick.getRawButton(5)) {
      IntakeSubsystem.intakeMotor.set(0.5);
      ShooterSubsystem.shooterWheels.set(-0.2);
    }

// Outtakes
    else if (RobotContainer.joystick.getRawButton(6)) {
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
