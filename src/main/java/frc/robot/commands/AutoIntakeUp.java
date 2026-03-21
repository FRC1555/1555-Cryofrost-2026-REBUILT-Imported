package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public final class AutoIntakeUp {
  private AutoIntakeUp() {}

  public static Command intakeUpSystem(IntakeSubsystem intakeSubsystem) {
    // Autonomous helper for driving the intake arm upward.
    return intakeSubsystem.setArmSpeedCommand(0.2);
  }
}
