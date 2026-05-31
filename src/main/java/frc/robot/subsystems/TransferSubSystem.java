package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Transfer subsystem — moves the note from the floor intake up to the shooter.
 *
 * <p>CAN ID: 2 (Spark Flex) — transferMotor
 */
public class TransferSubSystem {

  public final static SparkFlex transferMotor = new SparkFlex(2, MotorType.kBrushless);
  public static double transferMotorSpeed = 0.02;

  public void Transfersystem() {
    transferMotor.set(transferMotorSpeed);
  }

  public void setTransferMotorSpeed(double newTransferMotorSpeed) {
    transferMotorSpeed = newTransferMotorSpeed;
  }
}
