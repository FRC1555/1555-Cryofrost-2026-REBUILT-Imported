package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Factory class for intake arm up commands. Note: extends Command is unnecessary here — these are
 * factory methods, not command overrides.
 */
public class AutoIntakeUp extends Command {

  /** Pivots the intake arm up at the given speed (positive speed = up based on mechanism). */
  public Command IntakeUpSystem(double IntakeArmSpeed) {
    return Commands.runOnce(() -> IntakeSubsystem.intakeMotorArm.set(IntakeArmSpeed));
  }

  /** Stops the intake arm. */
  public Command IntakeUpOff() {
    return Commands.runOnce(() -> IntakeSubsystem.intakeMotorArm.set(0));
  }
}
