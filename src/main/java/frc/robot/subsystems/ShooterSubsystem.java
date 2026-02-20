package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem {

    private final SparkFlex ShooterMotorleft = new SparkFlex(18, MotorType.kBrushless);// Follower
    public final SparkFlex ShooterMotorRight = new SparkFlex(19, MotorType.kBrushless); // Leader

    public ShooterSubsystem() {
        double ShooterMotorRightspeed = 1.0;

        ShooterMotorRight.set(ShooterMotorRightspeed);
    }
}