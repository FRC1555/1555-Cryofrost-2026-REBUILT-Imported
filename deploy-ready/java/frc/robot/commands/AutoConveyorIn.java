package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBeltSubsystem;

/** Static factories for conveyor forward autonomous actions. */
public final class AutoConveyorIn {
  private AutoConveyorIn() {}

  public static Command conveyorIn(ConveyorBeltSubsystem conveyorBeltSubsystem) {
    // Autonomous helper for feeding a game piece toward the shooter.
    return conveyorBeltSubsystem.setConveyorSpeedCommand(1.0);
  }
}
