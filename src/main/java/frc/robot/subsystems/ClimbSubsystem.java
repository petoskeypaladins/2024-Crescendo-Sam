// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  public CANSparkMax leftClimb = new CANSparkMax(DriveConstants.leftWinch, MotorType.kBrushless);
  public CANSparkMax rightClimb = new CANSparkMax(DriveConstants.rightWinch, MotorType.kBrushless);


  public ClimbSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void leftUp() {
    if (RobotContainer.joystick.getRawButton(5)) {
      leftClimb.set(RobotContainer.joystick2.getRawAxis(3));
    }
    else {
      leftClimb.set(0);
    }
  }
  
  public void leftUpFast() {
    leftClimb.set(0.8);
  }

  public void leftDown() {
    leftClimb.set(-0.5);
  }

  public void rightUp() {
    if (RobotContainer.joystick.getRawButton(6)) {
      rightClimb.set(RobotContainer.joystick2.getRawAxis(3) * -1);
    }
    else {
      rightClimb.set(0);
    }
  }

  public void rightUpFast() {
    rightClimb.set(-0.8);
  }

  public void rightDown() {
    rightClimb.set(0.5);
  }

  public void stopLeft() {
    leftClimb.set(0);
  }

  public void stopRight() {
    rightClimb.set(0);
  }
}
