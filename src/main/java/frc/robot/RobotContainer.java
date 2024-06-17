// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonMoveForward;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LeftBackwardClimbCommand;
import frc.robot.commands.LeftClimbFastCommand;
import frc.robot.commands.LeftForwardClimbCommand;
import frc.robot.commands.MoveServoCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.PivotDownCommand;
import frc.robot.commands.PivotUpCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.RightBackwardClimbCommand;
import frc.robot.commands.RightClimbFastCommand;
import frc.robot.commands.RightForwardClimbCommand;
import frc.robot.commands.ServoAngleCommand;
import frc.robot.commands.ServoCommand;
import frc.robot.commands.ShooterAmpCommand;
import frc.robot.commands.ShooterSpeakerCommand;
import frc.robot.commands.SpeakerPresetCommand;
import frc.robot.commands.StopperCommand;
import frc.robot.commands.StopperReturnCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.StopperSubsystem;
// import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.USB0Camera;
import frc.robot.subsystems.USB1Camera;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  public static final ServoSubsystem m_ServoSubsystem = new ServoSubsystem();
  public static final StopperSubsystem m_StopperSubsystem = new StopperSubsystem();

  public final TeleopDriveCommand m_DriveCommand = new TeleopDriveCommand();
  public final IntakeCommand m_IntakeCommand = new IntakeCommand();
  public final ShooterSpeakerCommand m_ShooterSpeakerCommand = new ShooterSpeakerCommand();
  public final ShooterAmpCommand m_ShooterAmpCommand = new ShooterAmpCommand();
  public final ServoAngleCommand m_ServoAngleCommand = new ServoAngleCommand(0);
  public final MoveServoCommand m_MoveServoCommand = new MoveServoCommand();
  public final PivotCommand m_PivotCommand = new PivotCommand();
  public final SpeakerPresetCommand m_SpeakerPresetCommand = new SpeakerPresetCommand();
  public final ServoCommand m_ServoCommand = new ServoCommand();
  public final OuttakeCommand m_OuttakeCommand = new OuttakeCommand();
  public final PivotDownCommand m_PivotDownCommand = new PivotDownCommand();
  public final ResetGyroCommand m_ResetGyroCommand = new ResetGyroCommand();
  public final PivotUpCommand m_PivotUpCommand = new PivotUpCommand();
  public final LeftForwardClimbCommand m_LeftForwardClimbCommand = new LeftForwardClimbCommand();
  public final RightForwardClimbCommand m_RightForwardClimbCommand = new RightForwardClimbCommand();
  public final RightBackwardClimbCommand m_RightBackwardClimbCommand = new RightBackwardClimbCommand();
  public final LeftBackwardClimbCommand m_LeftBackwardClimbCommand = new LeftBackwardClimbCommand();
  public final LeftClimbFastCommand m_LeftClimbFastCommand = new LeftClimbFastCommand();
  public final RightClimbFastCommand m_RightClimbFastCommand = new RightClimbFastCommand();
  public final StopperCommand m_StopperCommand = new StopperCommand();
  public final StopperReturnCommand m_StopperReturnCommand = new StopperReturnCommand();

    
  

  //Autonomous Commands
  public final AutonMoveForward m_AutonMoveForward = new AutonMoveForward();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

      public static XboxController xboxController = new XboxController(0);
      public static Joystick joystick = new Joystick(1);
      public static CommandJoystick joystick2 = new CommandJoystick(1);
  public static final USB0Camera M_USB0CAMERA = new USB0Camera();


  private final SendableChooser<Command> autoChooser;
  public static final USB1Camera M_USB1CAMERA = new USB1Camera();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();


    NamedCommands.registerCommand("Intake Command", m_IntakeCommand);
    NamedCommands.registerCommand("Intake Stop Command", Commands.runOnce(() -> m_IntakeSubsytem.stop(), m_IntakeSubsytem));
    NamedCommands.registerCommand("Speaker Preset", m_SpeakerPresetCommand);
    NamedCommands.registerCommand("Speaker Rev", m_ShooterSpeakerCommand);
    NamedCommands.registerCommand("Shoot Command", Commands.runOnce(() -> m_ServoSubsystem.moveServo(), m_ServoSubsystem));
    NamedCommands.registerCommand("Stop Rev", Commands.runOnce(() -> m_ShooterSubsystem.stop(), m_ShooterSubsystem));
    NamedCommands.registerCommand("Pivot Down", m_PivotDownCommand);
    NamedCommands.registerCommand("Reset Servo", Commands.runOnce(() -> m_ServoSubsystem.resetServo(), m_ServoSubsystem));
    NamedCommands.registerCommand("Reset Gyro", Commands.runOnce(() -> m_DriveSubsystem.resetGyro(0), m_DriveSubsystem));
    NamedCommands.registerCommand("Reset Gyro 120", Commands.runOnce(() -> m_DriveSubsystem.resetGyro(120), m_DriveSubsystem));
    NamedCommands.registerCommand("Pivot Up", m_PivotUpCommand);
    NamedCommands.registerCommand("Engage Stopper", Commands.runOnce(() -> m_StopperSubsystem.resetStopper(), m_ShooterSubsystem));
    NamedCommands.registerCommand("Reset Stopper", Commands.runOnce(() -> m_StopperSubsystem.moveStopper(), m_ShooterSubsystem));
    NamedCommands.registerCommand("Sit There", Commands.runOnce(() -> m_DriveSubsystem.nothing(), m_DriveSubsystem));

   autoChooser = AutoBuilder.buildAutoChooser();
    
   SmartDashboard.putData("Select Auto", autoChooser);

   
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
    m_DriveSubsystem.setDefaultCommand(m_DriveCommand);


     // Intake
     m_driverController.rightBumper().whileTrue(m_IntakeCommand);
     
    // Shoots Speaker
    joystick2.button(1).whileTrue(m_ShooterSpeakerCommand);

    // Shoots Amp (Outtake)
    joystick2.button(11).whileTrue(m_ShooterAmpCommand);
    joystick2.button(11).whileTrue(m_OuttakeCommand);

    // Pivots
    joystick2.axisGreaterThan(1, 0.02).whileTrue(m_PivotCommand);
    joystick2.axisLessThan(1, -0.02).whileTrue(m_PivotCommand);

    // Speaker Preset
    joystick2.button(7).whileTrue(m_SpeakerPresetCommand);

    // Moves Servo
    joystick2.button(2).whileTrue(m_ServoCommand);

    //Moves Stopper Out
    joystick2.button(9).whileTrue(m_StopperReturnCommand);

    //Moves Stopper Back
    joystick2.button(10).whileTrue(m_StopperCommand);

    // Resets Gyro
    m_driverController.povDown().whileTrue(m_ResetGyroCommand);

    // Left Climb Forward
     joystick2.axisGreaterThan(3, 0).whileTrue(m_RightForwardClimbCommand);

    // Right Climb Forward
    joystick2.axisGreaterThan(3, 0).whileTrue(m_LeftForwardClimbCommand);



    // Left Climb Backward
    //joystick2.button(3).whileTrue(m_LeftBackwardClimbCommand);

    // Right Climb Backward
    //joystick2.button(4).whileTrue(m_RightBackwardClimbCommand);


  
    
    


    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Intakes
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

    return autoChooser.getSelected();
        
  } 
}
