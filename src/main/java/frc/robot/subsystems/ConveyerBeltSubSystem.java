package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ConveyerBeltSubSystem {
    public static final SparkFlex ConveyerMotor = new SparkFlex(3, MotorType.kBrushless);

        public static double ConveyerSpeed = 0.02;      
    
        // Command a target position in *rotations* of the motor shaft
        

    public void ConveyerSystem(){
    ConveyerMotor.set(ConveyerSpeed);
  }
    public void setintakeMotorSpeed(double newConveyerMotorSpeed){
    ConveyerSpeed = newConveyerMotorSpeed;
  };
  
    
}
