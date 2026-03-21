package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Conveyor control used to move game pieces between the intake and shooter. */
public class ConveyorBeltSubsystem extends SubsystemBase {
  // Conveyor motor used to move a game piece into or away from the shooter.
  private final SparkMax conveyorMotor = new SparkMax(3, MotorType.kBrushless);
  private double conveyorSpeed = 0.0;

  public void setConveyorMotorSpeed(double newConveyorMotorSpeed) {
    // Apply conveyor speed immediately.
    conveyorSpeed = newConveyorMotorSpeed;
    conveyorMotor.set(conveyorSpeed);
  }

  public void stopConveyorMotor() {
    // Shared conveyor stop helper.
    setConveyorMotorSpeed(0.0);
  }

  public Command runConveyorCommand(double speed) {
    // Hold a conveyor speed while the trigger command remains scheduled.
    return Commands.startEnd(() -> setConveyorMotorSpeed(speed), this::stopConveyorMotor, this);
  }

  public Command setConveyorSpeedCommand(double speed) {
    // Latch conveyor speed for auto sequences.
    return Commands.runOnce(() -> setConveyorMotorSpeed(speed), this);
  }

  public Command stopConveyorCommand() {
    // One-shot conveyor stop for cleanup steps.
    return Commands.runOnce(this::stopConveyorMotor, this);
  }
}
