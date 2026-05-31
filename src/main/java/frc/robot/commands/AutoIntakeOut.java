package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * TODO: This command is BROKEN — the IntakeOutSystem() method has an infinite while(true) loop with
 * no return statement, no timeout, and no interrupt condition. It will hang the robot if called.
 * It does not conform to the Command interface (extends Command but never returns a Command).
 * Either remove this file or rewrite it using Commands.runOnce() like the other auto commands.
 */
public class AutoIntakeOut extends Command {
  public static Command IntakeOutSystem() {
    while (true) {
      IntakeSubsystem.intakeMotor.set(-0.45);
    }
  }
}
