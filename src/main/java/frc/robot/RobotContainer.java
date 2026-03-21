// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Max smells bad
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.event.BooleanEvent;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubSystem2026Rebuilt;
import java.util.List;
import java.util.Queue;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubSystem;
import frc.robot.subsystems.ConveyerBeltSubSystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AutoConveyerIn;
import frc.robot.commands.AutoIntakeDown;
import frc.robot.commands.AutoIntakeIn;
import frc.robot.commands.AutoIntakeOut;
import frc.robot.commands.AutoIntakeUp;
import frc.robot.commands.AutoShoot;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems9
   
//   private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
//   private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final AutoShoot m_autoShoot = new AutoShoot();
    private final VisionSubSystem2026Rebuilt m_visionSubsystem = new VisionSubSystem2026Rebuilt("RightCAM");
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionSubsystem);
    private final AutoConveyerIn m_AutoConveyerIn = new AutoConveyerIn();
    private final AutoIntakeIn m_AutoIntakeIn = new AutoIntakeIn();
    private final AutoIntakeDown m_AutoIntakeDown = new AutoIntakeDown();
    private final AutoIntakeUp m_AutoIntakeUp = new AutoIntakeUp();

  // The driver's controlleo
  public Joystick m_driverController =
      new Joystick(OIConstants.kDriverControllerPort);
  public CommandXboxController m_manipController =
      new CommandXboxController(OIConstants.kManipControllerPort);

  // Joystick buttons for speed control
  public JoystickButton fullSendButton = new JoystickButton(m_driverController, 1);
  public JoystickButton highSpeedButton = new JoystickButton(m_driverController, 2);
  public JoystickButton midSpeedButton = new JoystickButton(m_driverController, 3);
  public JoystickButton lowSpeedButton = new JoystickButton(m_driverController, 4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("ShootOn", m_autoShoot.ShootOn(0.62));
    NamedCommands.registerCommand("ShootOff", m_autoShoot.ShootOff());
    NamedCommands.registerCommand("IntakeDownOn", m_AutoIntakeDown.IntakeDownOn(-0.2));
    NamedCommands.registerCommand("IntakeDownOff", m_AutoIntakeDown.IntakeDownOff());
    NamedCommands.registerCommand("IntakeUpSystem", m_AutoIntakeUp);
    NamedCommands.registerCommand("IntakeInSystem", m_AutoIntakeIn.IntakeInOn(0.45));
    NamedCommands.registerCommand("IntakeOffSystem", m_AutoIntakeIn.IntakeOff());
    NamedCommands.registerCommand("ConveyerIn", m_AutoConveyerIn.ConveyerIn(-1.3));
    NamedCommands.registerCommand("ConveyerOff", m_AutoConveyerIn.ConveyerOff());


    //building the auto chooser on smartdashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();



    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getZ(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Shooter Subsystem
    m_manipController.rightTrigger().whileTrue(new RunCommand(() -> m_shooterSubsystem.ShooterMotorRight.set(0.55)));
    m_manipController.rightTrigger().onFalse(new RunCommand(() -> m_shooterSubsystem.ShooterMotorRight.set(0.0)));

    //m_manipController.back().whileTrue(new RunCommand(() -> AutoShoot.Shoot()));

    m_manipController.leftTrigger().whileTrue(new RunCommand(() -> m_shooterSubsystem.ShooterMotorRight.set(-0.6)));
    m_manipController.leftTrigger().onFalse(new RunCommand(() -> m_shooterSubsystem.ShooterMotorRight.set(0)));

    //run intake in
    m_manipController.rightBumper().whileTrue(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = 0.45)));
    m_manipController.rightBumper().onFalse(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = 0.00)));

    m_manipController.a().whileTrue(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = 0.45)));
    m_manipController.a().onFalse(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = 0.00)));

    //full send intake
    m_manipController.start().whileTrue(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = 1))); 
    m_manipController.start().onFalse(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = 0.00)));

    //Run intake out
    m_manipController.y().whileTrue(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = -0.45)));
    m_manipController.y().onFalse(new RunCommand(() -> IntakeSubsystem.intakeMotor.set(IntakeSubsystem.intakeMotorSpeed = -0.00)));

    //run arm in
    m_manipController.b().whileTrue(new RunCommand(() -> IntakeSubsystem.intakeMotorArm.set(IntakeSubsystem.IntakeArmAngleSpeed = -0.2)));
    m_manipController.b().onFalse(new RunCommand(() -> IntakeSubsystem.intakeMotorArm.set(IntakeSubsystem.IntakeArmAngleSpeed = 0.00)));

    //Run arm out
    m_manipController.x().whileTrue(new RunCommand(() -> IntakeSubsystem.intakeMotorArm.set(IntakeSubsystem.IntakeArmAngleSpeed = 0.2)));
    m_manipController.x().onFalse(new RunCommand(() -> IntakeSubsystem.intakeMotorArm.set(IntakeSubsystem.IntakeArmAngleSpeed = 0.00)));

    //Conveyer Motor in
    // 2-28-2026, Fixed Bumper issue. What was it? Needs to be whileTrue. Not onTrue.
    m_manipController.leftBumper().whileTrue(new RunCommand(() -> ConveyerBeltSubSystem.ConveyerMotor.set(ConveyerBeltSubSystem.ConveyerSpeed = 1.3)));
    m_manipController.leftBumper().onFalse(new RunCommand(() -> ConveyerBeltSubSystem.ConveyerMotor.set(ConveyerBeltSubSystem.ConveyerSpeed = 0.0)));

    //Conveyer Motor out
    m_manipController.rightBumper().whileTrue(new RunCommand(() -> ConveyerBeltSubSystem.ConveyerMotor.set(ConveyerBeltSubSystem.ConveyerSpeed = -1.3)));
    m_manipController.rightBumper().onFalse(new RunCommand(() -> ConveyerBeltSubSystem.ConveyerMotor.set(ConveyerBeltSubSystem.ConveyerSpeed = 0.0)));
    

    // Turtle Beach Button 1 or Colored A -> Set Robot Speed to Full Send (Child Detected)
    fullSendButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(1)));
    highSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.75)));
    midSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.5)));
    lowSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.25)));

        // Bind buttons 5, 6, 7, and 8 to reset the gyro
        new JoystickButton(m_driverController, 5)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
        new JoystickButton(m_driverController, 6)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
        new JoystickButton(m_driverController, 7)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
        new JoystickButton(m_driverController, 8)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private final SendableChooser<Command> autoChooser;
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}