package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Factory class for shooter auto commands. This is the correct pattern — a plain class (not
 * extending Command) that returns Commands via factory methods.
 */
public class AutoShoot {
  private final ShooterSubsystem shooterSubsystem;

  public AutoShoot(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  /** Runs the shooter flywheels at the given speed once. */
  public Command ShootOn(double ShootSpeed) {
    return Commands.runOnce(() -> shooterSubsystem.setShooterMotorSpeed(ShootSpeed));
  }

  /** Stops the shooter flywheels. */
  public Command ShootOff() {
    return Commands.runOnce(() -> shooterSubsystem.setShooterMotorSpeed(0));
  }
}
