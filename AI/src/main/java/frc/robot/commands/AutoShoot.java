package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Static factories for shooter-related PathPlanner named commands. */
public final class AutoShoot {
  private AutoShoot() {}

  public static Command shootOn(ShooterSubsystem shooterSubsystem, double shootSpeed) {
    // Latch shooter speed so the wheel stays spinning after the named command finishes.
    return shooterSubsystem.setShooterSpeedCommand(shootSpeed);
  }

  public static Command shootOff(ShooterSubsystem shooterSubsystem) {
    // Matching stop command used after the feed step completes.
    return shooterSubsystem.stopShooterCommand();
  }
}
