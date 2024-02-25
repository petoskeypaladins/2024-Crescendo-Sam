// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonMoveForward;
import frc.robot.commands.ClimbCommand;
// import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveServoCommand;
import frc.robot.commands.ServoAngleCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.USB0Camera;
//import frc.robot.subsystems.USB1Camera;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  public static final IntakeSubsystem m_IntakeSubsytem = new IntakeSubsystem();
  public static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public static final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  public final TeleopDriveCommand m_DriveCommand = new TeleopDriveCommand();
  public final IntakeCommand m_IntakeCommand = new IntakeCommand();
  public final ShooterCommand m_ShooterCommand = new ShooterCommand();
  public final ServoAngleCommand m_ServoAngleCommand = new ServoAngleCommand(0);
  public final MoveServoCommand m_MoveServoCommand = new MoveServoCommand();
  public final ClimbCommand m_ClimbCommand = new ClimbCommand();
  //Autonomous Commands
  public final AutonMoveForward m_AutonMoveForward = new AutonMoveForward();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

      public static XboxController xboxController = new XboxController(0);
      public static Joystick joystick = new Joystick(1);
  public static final USB0Camera M_USB0CAMERA = new USB0Camera();
  //public static final USB1Camera M_USB1CAMERA = new USB1Camera();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);

    return m_AutonMoveForward;
        
  } 
}
