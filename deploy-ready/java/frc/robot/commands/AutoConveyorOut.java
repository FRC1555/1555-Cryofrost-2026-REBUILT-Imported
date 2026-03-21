package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBeltSubsystem;

/** Static factories for conveyor reverse autonomous actions. */
public final class AutoConveyorOut {
  private AutoConveyorOut() {}

  public static Command conveyorOut(ConveyorBeltSubsystem conveyorBeltSubsystem) {
    // Autonomous helper for reversing the conveyor to clear or back out a piece.
    return conveyorBeltSubsystem.setConveyorSpeedCommand(-1.0);
  }
}
