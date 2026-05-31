package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ConveyerBeltSubSystem;

/**
 * Factory class for conveyer belt auto commands. Note: extends Command is unnecessary here — these
 * are factory methods, not command overrides. AutoShoot does it correctly (plain class).
 */
public class AutoConveyerIn extends Command {

  /** Runs ConveyerMotor at the given speed once. Positive = conveyer in. */
  public Command ConveyerIn(double ConveyerSpeed) {
    return Commands.runOnce(() -> ConveyerBeltSubSystem.ConveyerMotor.set(ConveyerSpeed));
  }

  /** Stops the conveyer motor. */
  public Command ConveyerOff() {
    return Commands.runOnce(() -> ConveyerBeltSubSystem.ConveyerMotor.set(0));
  }
}
