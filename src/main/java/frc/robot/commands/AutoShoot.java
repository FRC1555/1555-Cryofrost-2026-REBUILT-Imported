package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShoot extends Command {

    public static void Shoot() {
    while (true) {
    ShooterSubsystem.ShooterMotorRight.set(0.62);
    
    }

}
}
