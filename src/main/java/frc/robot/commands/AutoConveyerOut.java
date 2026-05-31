package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerBeltSubSystem;

/**
 * TODO: This command is BROKEN — the TransferOut() method has an infinite while(true) loop with
 * no return statement, no timeout, and no interrupt condition. It will hang the robot if called.
 * Same issue as AutoIntakeOut. Either remove or rewrite using Commands.runOnce().
 */
public class AutoConveyerOut extends Command {
  public static Command TransferOut() {
    while (true) {
      ConveyerBeltSubSystem.ConveyerMotor.set(-1.3);
    }
  }
}
