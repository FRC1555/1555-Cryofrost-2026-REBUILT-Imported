package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeOut extends Command{
            public static Command IntakeOutSystem(){
      while (true){
        IntakeSubsystem.intakeMotor.set(-0.45);
      }
    }
}
