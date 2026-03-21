package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerBeltSubSystem;

/** Static factories for conveyor reverse autonomous actions. */
public final class AutoConveyerOut {
  private AutoConveyerOut() {}

  public static Command conveyerOut(ConveyerBeltSubSystem conveyerBeltSubSystem) {
    // Autonomous helper for reversing the conveyor to clear or back out a piece.
    return conveyerBeltSubSystem.setConveyerSpeedCommand(-1.0);
  }
}
