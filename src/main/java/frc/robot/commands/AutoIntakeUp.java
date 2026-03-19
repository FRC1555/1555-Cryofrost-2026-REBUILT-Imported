package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeUp extends Command{
        public static Command IntakeUpSystem(){
    while (true){
      IntakeSubsystem.intakeMotorArm.set(0.2);
    }
  }
}
