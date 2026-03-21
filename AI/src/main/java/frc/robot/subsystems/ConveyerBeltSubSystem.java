package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Conveyor control used to move game pieces between the intake and shooter. */
public class ConveyerBeltSubSystem extends SubsystemBase {
  // Conveyor motor used to move a game piece into or away from the shooter.
  private final SparkMax conveyerMotor = new SparkMax(3, MotorType.kBrushless);
  private double conveyerSpeed = 0.0;

  public void setConveyerMotorSpeed(double newConveyerMotorSpeed) {
    // Apply conveyor speed immediately.
    conveyerSpeed = newConveyerMotorSpeed;
    conveyerMotor.set(conveyerSpeed);
  }

  public void stopConveyerMotor() {
    // Shared conveyor stop helper.
    setConveyerMotorSpeed(0.0);
  }

  public Command runConveyerCommand(double speed) {
    // Hold a conveyor speed while the trigger command remains scheduled.
    return Commands.startEnd(() -> setConveyerMotorSpeed(speed), this::stopConveyerMotor, this);
  }

  public Command setConveyerSpeedCommand(double speed) {
    // Latch conveyor speed for auto sequences.
    return Commands.runOnce(() -> setConveyerMotorSpeed(speed), this);
  }

  public Command stopConveyerCommand() {
    // One-shot conveyor stop for cleanup steps.
    return Commands.runOnce(this::stopConveyerMotor, this);
  }
}
