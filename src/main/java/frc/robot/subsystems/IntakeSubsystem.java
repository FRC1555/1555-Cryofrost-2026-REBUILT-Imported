package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem {
    
    public final static SparkFlex intakeMotorArm = new SparkFlex(16, MotorType.kBrushless);
    public final static SparkFlex intakeMotor = new SparkFlex(17, MotorType.kBrushless);


        public static double intakeMotorSpeed = 0.02;

        public static double IntakeArmAngleSpeed = 0.02;       
    
        // Command a target position in *rotations* of the motor shaft
        

    public void IntakeSystem(){
    intakeMotor.set(intakeMotorSpeed);
  }
    public void setintakeMotorSpeed(double newIntakeMotorSpeed){
    intakeMotorSpeed = newIntakeMotorSpeed;
  };
  
//   public void setIntakeArmAngleDegrees(double newAngleDegrees){
//     IntakeArmAngleDegrees = newAngleDegrees;
//   }

    public void IntakeArmSystem(){
    intakeMotorArm.set(IntakeArmAngleSpeed);
}

  public static Command IntakeAutoSystem(){
    while (true){
      intakeMotorArm.set(IntakeArmAngleSpeed);
    }
  }
}
