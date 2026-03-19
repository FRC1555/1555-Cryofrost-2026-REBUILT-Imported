package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeDown extends Command{
      public static Command IntakeDownSystem(){
    while (true){
      IntakeSubsystem.intakeMotorArm.set(-0.2);
    }
  }
}
