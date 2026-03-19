package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerBeltSubSystem;

public class AutoConveyerOut extends Command {
        public static Command TransferOut(){
    while (true){
      ConveyerBeltSubSystem.ConveyerMotor.set(-1.3);
    }
  }
}
