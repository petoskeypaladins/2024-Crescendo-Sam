// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

 public CANSparkMax shooterWheels = new CANSparkMax(DriveConstants.shooter, MotorType.kBrushless);
 private CANSparkMax pivotMotor = new CANSparkMax(DriveConstants.shooterPivot, MotorType.kBrushed);

 public Servo servo = new Servo(0);
 //Servo Stopper:Added by Cosmo
 public Servo Stopper = new Servo(1);

 
  public DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  //do the green box on the smart dashboard
  boolean shooterInRange;
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //293.4
    if (.2934 < pivotEncoder.getAbsolutePosition() && pivotEncoder.getAbsolutePosition() < .3652){
      shooterInRange = true;
    } else {
      shooterInRange = false;
    }
     SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Ready For Preset?", shooterInRange);
    }


  public void shooterIntake() {
    shooterWheels.set(-0.3);
  }

  public void speaker() {
    shooterWheels.set(0.8);
  }

  public void amp() {
    shooterWheels.set(0.21);
  }
//this one is for TeleOp, all the rest are for auton!!! 
  public void pivot() {
    pivotMotor.set(-1*RobotContainer.joystick2.getRawAxis(1));
  }

  public void pivotDown() {
    pivotMotor.set(0.8);
  }

  public void pivotUp() {
    pivotMotor.set(-0.8);
  }
  public void stop() {
    shooterWheels.set(0);
  }

  public void stopPivot() {
    pivotMotor.set(0);
  }

// Original is -0.18
  // Original is 326 < x < 332
  public void speakerPreset() {
    if(RobotContainer.m_ShooterSubsystem.pivotEncoder.getAbsolutePosition() < 0.324) {
      RobotContainer.m_ShooterSubsystem.pivotMotor.set(-0.18 - (0.326 - RobotContainer.m_ShooterSubsystem.pivotEncoder.getAbsolutePosition()));
    }
    else if (RobotContainer.m_ShooterSubsystem.pivotEncoder.getAbsolutePosition() > 0.330) {
      RobotContainer.m_ShooterSubsystem.pivotMotor.set(0.18 + (RobotContainer.m_ShooterSubsystem.pivotEncoder.getAbsolutePosition() - 0.332));
    }
    else {
      RobotContainer.m_ShooterSubsystem.pivotMotor.set(0);
    }
  }

  public void speakerPreset2PieceSpeaker() {
  }


  
}
