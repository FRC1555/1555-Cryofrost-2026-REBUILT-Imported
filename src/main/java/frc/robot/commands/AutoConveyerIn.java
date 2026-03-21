package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerBeltSubSystem;

public final class AutoConveyerIn {
  private AutoConveyerIn() {}

  public static Command conveyerIn(ConveyerBeltSubSystem conveyerBeltSubSystem) {
    // Autonomous helper for feeding a game piece toward the shooter.
    return conveyerBeltSubSystem.setConveyerSpeedCommand(1.0);
  }
}
