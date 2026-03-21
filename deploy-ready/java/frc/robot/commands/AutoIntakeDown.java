package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Static factories for intake arm downward autonomous actions. */
public final class AutoIntakeDown {
  private AutoIntakeDown() {}

  public static Command intakeDownSystem(IntakeSubsystem intakeSubsystem) {
    // Autonomous helper for driving the intake arm downward.
    return intakeSubsystem.setArmSpeedCommand(-0.2);
  }
}
