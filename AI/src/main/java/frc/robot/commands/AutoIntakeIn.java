package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Static factories for intake roller forward autonomous actions. */
public final class AutoIntakeIn {
  private AutoIntakeIn() {}

  public static Command intakeInSystem(IntakeSubsystem intakeSubsystem) {
    // Autonomous helper for turning the intake roller inward.
    return intakeSubsystem.setIntakeSpeedCommand(0.45);
  }
}
