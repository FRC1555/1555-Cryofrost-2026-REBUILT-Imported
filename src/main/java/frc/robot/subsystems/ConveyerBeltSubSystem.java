package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ConveyerBeltSubSystem {
    public static final SparkMax ConveyerMotor = new SparkMax(3, MotorType.kBrushless);

        public static double ConveyerSpeed = 0.02; 
        
        
    
        // Command a target position in *rotations* of the motor shaft
        

    public void ConveyerSystem(){
    ConveyerMotor.set(ConveyerSpeed);
  }
    public void setConveyerMotorSpeed(double newConveyerMotorSpeed){
    ConveyerSpeed = newConveyerMotorSpeed;


  };
  
    
}
