package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Conveyer belt subsystem — moves the note from the shooter back toward the intake (or vice versa)
 * to assist with feeding.
 *
 * <p>CAN ID: 3 (Spark Flex) — ConveyerMotor
 */
public class ConveyerBeltSubSystem {

  public static final SparkFlex ConveyerMotor = new SparkFlex(3, MotorType.kBrushless);
  public static double ConveyerSpeed = 0.02;

  public void ConveyerSystem() {
    ConveyerMotor.set(ConveyerSpeed);
  }

  public void setConveyerMotorSpeed(double newConveyerMotorSpeed) {
    ConveyerSpeed = newConveyerMotorSpeed;
  }
}
