package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Factory class for intake arm down commands. Note: extends Command is unnecessary here — these are
 * factory methods, not command overrides.
 */
public class AutoIntakeDown extends Command {

  /** Pivots the intake arm down at the given speed (negative speed = down based on mechanism). */
  public Command IntakeDownOn(double IntakeArmSpeed) {
    return Commands.runOnce(() -> IntakeSubsystem.intakeMotorArm.set(IntakeArmSpeed));
  }

  /** Stops the intake arm. */
  public Command IntakeDownOff() {
    return Commands.runOnce(() -> IntakeSubsystem.intakeMotorArm.set(0));
  }
}
