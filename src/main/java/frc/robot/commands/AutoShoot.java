package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class AutoShoot {
    private final ShooterSubsystem shooterSubsystem;

    public AutoShoot(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    public Command ShootOn(double ShootSpeed) {
        return Commands.runOnce(
            () -> shooterSubsystem.setShooterMotorSpeed(ShootSpeed)
        );
    }

    public Command ShootOff() {
        return Commands.runOnce(
            () -> shooterSubsystem.stopShooter()
        );
    }

}
