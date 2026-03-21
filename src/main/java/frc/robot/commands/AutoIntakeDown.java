package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeDown extends Command{
    public Command IntakeDownOn(double IntakeArmSpeed) {
        return Commands.runOnce(
            () -> IntakeSubsystem.intakeMotorArm.set(IntakeArmSpeed)
        );
    }
    public Command IntakeDownOff() {
        return Commands.runOnce(
            () -> IntakeSubsystem.intakeMotorArm.set(0)
        );
    }
}
