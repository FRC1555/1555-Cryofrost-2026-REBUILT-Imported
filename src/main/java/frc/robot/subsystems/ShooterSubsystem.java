package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem {

    private final SparkFlex ShooterMotorleft = new SparkFlex(18, MotorType.kBrushless);// Follower
    public static SparkFlex ShooterMotorRight = new SparkFlex(19, MotorType.kBrushless); // Leader

    public double ShooterMotorRightspeed = 0.62;
    
    public void Shootersystem() {
        ShooterMotorRight.set(ShooterMotorRightspeed);
    }
    public void setShooterMotorSpeed(double newShooterMotorSpeed){
    ShooterMotorRightspeed = newShooterMotorSpeed;
  };
}