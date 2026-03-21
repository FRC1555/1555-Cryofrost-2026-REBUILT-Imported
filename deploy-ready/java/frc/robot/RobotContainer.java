// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoConveyorIn;
import frc.robot.commands.AutoConveyorOut;
import frc.robot.commands.AutoIntakeDown;
import frc.robot.commands.AutoIntakeIn;
import frc.robot.commands.AutoIntakeOut;
import frc.robot.commands.AutoIntakeUp;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.ConveyorBeltSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubSystem2026Rebuilt;

/** Central wiring for the current 1555 robot subsystems, controls, and autonomous chooser. */
public class RobotContainer {
  // Core robot subsystems used by both teleop controls and autonomous named commands.
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ConveyorBeltSubsystem m_conveyorBeltSubsystem = new ConveyorBeltSubsystem();
  private final VisionSubSystem2026Rebuilt m_visionSubsystem =
      new VisionSubSystem2026Rebuilt("RightCAM");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionSubsystem);

  // PathPlanner auto-name chooser published to the dashboard.
  private final SendableChooser<String> autoChooser;

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
    autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Connect controller buttons to subsystem-owned commands.
    configureButtonBindings();

    // Default drive command: field-relative swerve controlled by the driver joystick.
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    // Most flight-style joysticks report forward/back on Y and left/right strafe on X.
                    MathUtil.applyDeadband(
                        m_driverController.getY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getZ(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
  }

  private SendableChooser<String> buildAutoChooser() {
    // Build a chooser from auto names so we can wrap the selected auto at runtime.
    SendableChooser<String> chooser = new SendableChooser<>();
    List<String> autoNames = new ArrayList<>(AutoBuilder.getAllAutoNames());
    autoNames.sort(Comparator.naturalOrder());

    if (autoNames.isEmpty()) {
      chooser.setDefaultOption("No Autos Found", "");
      return chooser;
    }

    String defaultAuto = autoNames.contains("Blue Auto Shot") ? "Blue Auto Shot" : autoNames.get(0);
    chooser.setDefaultOption(defaultAuto, defaultAuto);

    for (String autoName : autoNames) {
      if (!autoName.equals(defaultAuto)) {
        chooser.addOption(autoName, autoName);
      }
    }

    return chooser;
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
    NamedCommands.registerCommand("ConveyorIn", AutoConveyorIn.conveyorIn(m_conveyorBeltSubsystem));
    NamedCommands.registerCommand(
        "ConveyorOut", AutoConveyorOut.conveyorOut(m_conveyorBeltSubsystem));

    // Explicit stop commands keep autos deterministic after a scoring sequence finishes.
    NamedCommands.registerCommand("IntakeStop", m_intakeSubsystem.stopIntakeCommand());
    NamedCommands.registerCommand("IntakeArmStop", m_intakeSubsystem.stopArmCommand());
    NamedCommands.registerCommand("ConveyorStop", m_conveyorBeltSubsystem.stopConveyorCommand());
  }

  private void configureButtonBindings() {
    // Debounce analog triggers slightly so noisy trigger values do not chatter the shooter.
    Trigger shooterForwardTrigger =
        m_manipController.rightTrigger(OIConstants.kTriggerButtonThreshold).debounce(0.05);
    Trigger shooterReverseTrigger =
        m_manipController.leftTrigger(OIConstants.kTriggerButtonThreshold).debounce(0.05);

    // Shooter trigger bindings: right trigger shoots, left trigger reverses to clear jams.
    shooterForwardTrigger.whileTrue(m_shooterSubsystem.runShooterCommand(0.62));
    shooterForwardTrigger.onFalse(m_shooterSubsystem.stopShooterCommand());

    shooterReverseTrigger.whileTrue(m_shooterSubsystem.runShooterCommand(-0.6));
    shooterReverseTrigger.onFalse(m_shooterSubsystem.stopShooterCommand());

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
    m_manipController.leftBumper().whileTrue(m_conveyorBeltSubsystem.runConveyorCommand(1.0));
    m_manipController.leftBumper().onFalse(m_conveyorBeltSubsystem.stopConveyorCommand());

    m_manipController.rightBumper().whileTrue(m_conveyorBeltSubsystem.runConveyorCommand(-1.0));
    m_manipController.rightBumper().onFalse(m_conveyorBeltSubsystem.stopConveyorCommand());

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
    // Build the auto at autonomous start so we can use the freshest AprilTag pose available.
    return buildAutonomousCommand(autoChooser.getSelected());
  }

  private Command buildAutonomousCommand(String autoName) {
    if (autoName == null || autoName.isBlank()) {
      SmartDashboard.putString("Auto/StartMode", "No auto selected");
      return Commands.none();
    }

    SmartDashboard.putString("Auto/Selected", autoName);

    try {
      PathPlannerAuto selectedAuto = new PathPlannerAuto(autoName);
      List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoName);

      if (pathGroup.isEmpty()) {
        SmartDashboard.putBoolean("Auto/HasAprilTagStartPose", false);
        SmartDashboard.putString("Auto/StartMode", "No path data, running auto directly");
        return selectedAuto;
      }

      Optional<Pose2d> estimatedStartPose = m_visionSubsystem.getEstimatedPose2d();
      SmartDashboard.putBoolean("Auto/HasAprilTagStartPose", estimatedStartPose.isPresent());

      if (estimatedStartPose.isEmpty()) {
        SmartDashboard.putString("Auto/StartMode", "No AprilTag pose, running auto directly");
        return selectedAuto;
      }

      Pose2d visionSeedPose = estimatedStartPose.get();
      Pose2d desiredAutoStartPose = selectedAuto.getStartingPose();
      PathConstraints startConstraints = pathGroup.get(0).getGlobalConstraints();

      SmartDashboard.putString("Auto/VisionSeedPose", visionSeedPose.toString());
      SmartDashboard.putString("Auto/DesiredStartPose", desiredAutoStartPose.toString());
      SmartDashboard.putString("Auto/StartMode", "AprilTag seed + pathfind to auto start");

      // Seed odometry from AprilTags, drive to the selected auto start pose, then run the auto.
      return Commands.sequence(
              Commands.runOnce(() -> m_robotDrive.resetPose(visionSeedPose), m_robotDrive),
              AutoBuilder.pathfindToPoseFlipped(desiredAutoStartPose, startConstraints),
              selectedAuto)
          .withName("AprilTagStart_" + autoName.replace(' ', '_'));
    } catch (Exception e) {
      SmartDashboard.putBoolean("Auto/HasAprilTagStartPose", false);
      SmartDashboard.putString("Auto/StartMode", "Auto fallback after build error");
      SmartDashboard.putString(
          "Auto/StartError",
          e.getClass().getSimpleName() + (e.getMessage() == null ? "" : ": " + e.getMessage()));
      return AutoBuilder.buildAuto(autoName);
    }
  }
}
