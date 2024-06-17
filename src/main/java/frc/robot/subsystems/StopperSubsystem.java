// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class StopperSubsystem extends SubsystemBase {
  public boolean isEngaged;
  /** Creates a new StopperSubsystem. */
  public StopperSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.m_ShooterSubsystem.Stopper.getPosition() == 0){
     isEngaged = false;
    } else { 
      isEngaged = true;
    }
     SmartDashboard.putBoolean("Stopper Engaged?", isEngaged);
    
  }

  public void moveStopper() {
   RobotContainer.m_ShooterSubsystem.Stopper.setPosition(0);
   System.out.println("Stopper Moving In");
  }
  public void resetStopper() {


    RobotContainer.m_ShooterSubsystem.Stopper.setPosition(1);
  System.out.println("Stopper Moving Out");
  }
}
