package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ConveyerBeltSubSystem;

public class AutoConveyerIn extends Command{
    public Command ConveyerIn(double ConveyerSpeed) {
        return Commands.runOnce(
            () -> ConveyerBeltSubSystem.ConveyerMotor.set(ConveyerSpeed)
        );
    }
    public Command ConveyerOff() {
        return Commands.runOnce(
            () -> ConveyerBeltSubSystem.ConveyerMotor.set(0)
        );
    }
}
