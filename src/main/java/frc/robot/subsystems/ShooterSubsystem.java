package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.config.ClosedLoopConfig;

public class ShooterSubsystem {
    private SparkMax ShooterMotorleft;
    private SparkMax ShooterMotorRight;

    public ShooterSubsystem() {
        // Create motors (brushless for NEO/550)
        ShooterMotorleft = new SparkMax(16, MotorType.kBrushless);   // Leader CAN ID
        ShooterMotorRight = new SparkMax(17, MotorType.kBrushless); // Follower CAN ID


        // Optional: Configure leader for velocity control (if desired)
        SparkMaxConfig shootRightConfig = new SparkMaxConfig();
        shootRightConfig
            .encoder()  // or .closedLoopRampRate() etc.
            .positionConversionFactor(1.0)  // Adjust for your units
            .velocityConversionFactor(1.0);
        shootRightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0001, 0.0, 0.0)  // Tune these gains!
            .outputRange(-1, 1);
        ShooterMotorleft.configure(shootRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower mirrors leader (inverted if needed for opposite direction)
        SparkMaxConfig shootLeftConfig = new SparkMaxConfig();
        ShooterMotorleft.follow(ShooterMotorRight);  // Or follow(leaderMotor, true) to invert direction
        // followerConfig.follow(leaderMotor, true);  // Example: invert follower
        ShooterMotorleft.configure(shootLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // To set speed (e.g., in periodic or command)
    public void setSpeed(double percentOutput) {
        ShooterMotorRight.set(percentOutput);  // -1.0 to 1.0 duty cycle
        // Follower automatically follows — no need to set it!
    }

    // For closed-loop velocity (setpoint in RPM, assuming conversion factor = 1)
    public void setVelocity(double rpm) {
        ShooterMotorRight.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
        // Follower mirrors the resulting duty cycle
    }
}