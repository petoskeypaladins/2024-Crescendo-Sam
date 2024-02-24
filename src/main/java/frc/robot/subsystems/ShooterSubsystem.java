// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

 public static CANSparkMax shooterWheels = new CANSparkMax(DriveConstants.shooter, MotorType.kBrushless);
 public static CANSparkMax pivotMotor = new CANSparkMax(DriveConstants.shooterPivot, MotorType.kBrushed);

 public static Servo servo = new Servo(0);

 
  public static DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getAbsolutePosition());
  }
}
