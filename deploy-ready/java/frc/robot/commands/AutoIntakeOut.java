package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Static factories for intake roller reverse autonomous actions. */
public final class AutoIntakeOut {
  private AutoIntakeOut() {}

  public static Command intakeOutSystem(IntakeSubsystem intakeSubsystem) {
    // Autonomous helper for reversing the intake roller.
    return intakeSubsystem.setIntakeSpeedCommand(-0.45);
  }
}
