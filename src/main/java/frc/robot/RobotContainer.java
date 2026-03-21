// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoConveyerIn;
import frc.robot.commands.AutoConveyerOut;
import frc.robot.commands.AutoIntakeDown;
import frc.robot.commands.AutoIntakeIn;
import frc.robot.commands.AutoIntakeOut;
import frc.robot.commands.AutoIntakeUp;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.ConveyerBeltSubSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubSystem2026Rebuilt;

public class RobotContainer {
  // Core robot subsystems used by both teleop controls and autonomous named commands.
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ConveyerBeltSubSystem m_conveyerBeltSubSystem = new ConveyerBeltSubSystem();
  private final VisionSubSystem2026Rebuilt m_visionSubsystem =
      new VisionSubSystem2026Rebuilt("RightCAM");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionSubsystem);

  // PathPlanner auto chooser published to the dashboard.
  private final SendableChooser<Command> autoChooser;

  // Driver and manipulator input devices.
  public final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  public final CommandXboxController m_manipController =
      new CommandXboxController(OIConstants.kManipControllerPort);

  // Driver speed mode shortcuts.
  public final JoystickButton fullSendButton = new JoystickButton(m_driverController, 1);
  public final JoystickButton highSpeedButton = new JoystickButton(m_driverController, 2);
  public final JoystickButton midSpeedButton = new JoystickButton(m_driverController, 3);
  public final JoystickButton lowSpeedButton = new JoystickButton(m_driverController, 4);

  public RobotContainer() {
    // Register PathPlanner named commands before building the chooser so autos can resolve them.
    registerNamedCommands();

    // Build and publish the autonomous chooser shown on the dashboard.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Connect controller buttons to subsystem-owned commands.
    configureButtonBindings();

    // Default drive command: field-relative swerve controlled by the driver joystick.
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

  private void registerNamedCommands() {
    // Shooter controls used by sequential autos.
    NamedCommands.registerCommand("ShootOn", AutoShoot.shootOn(m_shooterSubsystem, 0.62));
    NamedCommands.registerCommand("ShootOff", AutoShoot.shootOff(m_shooterSubsystem));

    // Intake arm and roller controls used by autos to acquire or stage game pieces.
    NamedCommands.registerCommand(
        "IntakeDownSystem", AutoIntakeDown.intakeDownSystem(m_intakeSubsystem));
    NamedCommands.registerCommand(
        "IntakeUpSystem", AutoIntakeUp.intakeUpSystem(m_intakeSubsystem));
    NamedCommands.registerCommand(
        "IntakeInSystem", AutoIntakeIn.intakeInSystem(m_intakeSubsystem));
    NamedCommands.registerCommand(
        "IntakeOutSystem", AutoIntakeOut.intakeOutSystem(m_intakeSubsystem));

    // Conveyor controls used to feed the shooter.
    NamedCommands.registerCommand("ConveyerIn", AutoConveyerIn.conveyerIn(m_conveyerBeltSubSystem));
    NamedCommands.registerCommand(
        "ConveyerOut", AutoConveyerOut.conveyerOut(m_conveyerBeltSubSystem));

    // Explicit stop commands keep autos deterministic after a scoring sequence finishes.
    NamedCommands.registerCommand("IntakeStop", m_intakeSubsystem.stopIntakeCommand());
    NamedCommands.registerCommand("IntakeArmStop", m_intakeSubsystem.stopArmCommand());
    NamedCommands.registerCommand("ConveyerStop", m_conveyerBeltSubSystem.stopConveyerCommand());
  }

  private void configureButtonBindings() {
    // Shooter trigger bindings: right trigger shoots, left trigger reverses to clear jams.
    m_manipController.rightTrigger().whileTrue(m_shooterSubsystem.runShooterCommand(0.62));
    m_manipController.rightTrigger().onFalse(m_shooterSubsystem.setShooterSpeedCommand(0.10));

    m_manipController.leftTrigger().whileTrue(m_shooterSubsystem.runShooterCommand(-0.6));
    m_manipController.leftTrigger().onFalse(m_shooterSubsystem.stopShooterCommand());

    // Intake roller controls: normal intake, full-send intake, and reverse.
    m_manipController.a().whileTrue(m_intakeSubsystem.runIntakeCommand(0.45));
    m_manipController.a().onFalse(m_intakeSubsystem.stopIntakeCommand());

    m_manipController.start().whileTrue(m_intakeSubsystem.runIntakeCommand(1.0));
    m_manipController.start().onFalse(m_intakeSubsystem.stopIntakeCommand());

    m_manipController.y().whileTrue(m_intakeSubsystem.runIntakeCommand(-0.45));
    m_manipController.y().onFalse(m_intakeSubsystem.stopIntakeCommand());

    // Intake arm controls: move the arm down or up while the button is held.
    m_manipController.b().whileTrue(m_intakeSubsystem.runArmCommand(-0.2));
    m_manipController.b().onFalse(m_intakeSubsystem.stopArmCommand());

    m_manipController.x().whileTrue(m_intakeSubsystem.runArmCommand(0.2));
    m_manipController.x().onFalse(m_intakeSubsystem.stopArmCommand());

    // Conveyor controls: feed forward or reverse while the bumper is held.
    m_manipController.leftBumper().whileTrue(m_conveyerBeltSubSystem.runConveyerCommand(1.0));
    m_manipController.leftBumper().onFalse(m_conveyerBeltSubSystem.stopConveyerCommand());

    m_manipController.rightBumper().whileTrue(m_conveyerBeltSubSystem.runConveyerCommand(-1.0));
    m_manipController.rightBumper().onFalse(m_conveyerBeltSubSystem.stopConveyerCommand());

    // Driver speed presets for practice and match conditions.
    fullSendButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(1.0)));
    highSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.75)));
    midSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.5)));
    lowSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.25)));

    // Any of these buttons resets the gyro heading back to field-forward.
    new JoystickButton(m_driverController, 5)
        .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
    new JoystickButton(m_driverController, 6)
        .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
    new JoystickButton(m_driverController, 7)
        .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
    new JoystickButton(m_driverController, 8)
        .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
  }

  public Command getAutonomousCommand() {
    // Return the dashboard-selected autonomous routine.
    return autoChooser.getSelected();
  }
}
