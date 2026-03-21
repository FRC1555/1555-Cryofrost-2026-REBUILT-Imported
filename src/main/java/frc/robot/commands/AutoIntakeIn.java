package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeIn extends Command{
    public Command IntakeInOn(double IntakeSpeed) {
        return Commands.runOnce(
            () -> IntakeSubsystem.intakeMotor.set(IntakeSpeed)
        );
    }
    public Command IntakeOff() {
        return Commands.runOnce(
            () -> IntakeSubsystem.intakeMotor.set(0)
        );
    }
  }
