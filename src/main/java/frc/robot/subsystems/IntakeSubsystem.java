// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
  /* Creates a new IntakeSubsystem. */

private CANSparkMax intakeMotor = new CANSparkMax(DriveConstants.intake, MotorType.kBrushless); 
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
intakeMotor.set(0.5);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void outtake() {
    intakeMotor.set(-0.5);
  }
}
