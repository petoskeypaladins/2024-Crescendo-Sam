// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ServoSubsystem extends SubsystemBase {
  /** Creates a new ServoSubsystem. */
  public ServoSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveServo() {
    RobotContainer.m_ShooterSubsystem.servo.setPosition(0);
  }

  public void resetServo() {
    RobotContainer.m_ShooterSubsystem.servo.setPosition(1);
  }
}
