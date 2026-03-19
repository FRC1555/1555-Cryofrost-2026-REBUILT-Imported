package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeIn extends Command{
        public static Command IntakeInSystem(){
      while (true){
        IntakeSubsystem.intakeMotor.set(0.45);
      }
    }
}
