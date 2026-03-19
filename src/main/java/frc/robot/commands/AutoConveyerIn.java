package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerBeltSubSystem;

public class AutoConveyerIn extends Command{
      public static Command ConveyerIn(){
    while (true){
      ConveyerBeltSubSystem.ConveyerMotor.set(1.3);
    }
  }
}
