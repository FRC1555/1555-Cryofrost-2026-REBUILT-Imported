package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class AutoShoot {

    public Command ShootOn(double ShootSpeed) {
        return Commands.runOnce(
            () -> ShooterSubsystem.ShooterMotorRight.set(ShootSpeed)
        );
    }

    public Command ShootOff() {
        return Commands.runOnce(
            () -> ShooterSubsystem.ShooterMotorRight.set(0)
        );
    }

}
