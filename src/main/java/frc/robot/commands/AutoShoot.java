package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShoot extends Command {

    public static Command Shoot() {
    while (true) {
    ShooterSubsystem.ShooterMotorRight.set(0.62);
    }
}
}
