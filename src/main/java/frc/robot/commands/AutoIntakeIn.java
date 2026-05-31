package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Factory class for intake roller commands. Note: extends Command is unnecessary here — these are
 * factory methods, not command overrides.
 */
public class AutoIntakeIn extends Command {

  /** Runs the intake rollers inward at the given speed. */
  public Command IntakeInOn(double IntakeSpeed) {
    return Commands.runOnce(() -> IntakeSubsystem.intakeMotor.set(IntakeSpeed));
  }

  /** Stops the intake rollers. */
  public Command IntakeOff() {
    return Commands.runOnce(() -> IntakeSubsystem.intakeMotor.set(0));
  }
}
