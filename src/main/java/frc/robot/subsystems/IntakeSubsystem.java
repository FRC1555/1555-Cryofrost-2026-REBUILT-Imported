package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // Roller and arm motors for the intake assembly.
  private final SparkFlex intakeMotorArm = new SparkFlex(16, MotorType.kBrushless);
  private final SparkFlex intakeMotor = new SparkFlex(17, MotorType.kBrushless);

  // Cached commanded outputs, useful if the implementation grows to closed-loop control later.
  private double intakeMotorSpeed = 0.0;
  private double intakeArmAngleSpeed = 0.0;

  public void setIntakeMotorSpeed(double newIntakeMotorSpeed) {
    // Apply roller speed immediately.
    intakeMotorSpeed = newIntakeMotorSpeed;
    intakeMotor.set(intakeMotorSpeed);
  }

  public void stopIntakeMotor() {
    // Stop only the roller motor.
    setIntakeMotorSpeed(0.0);
  }

  public void setIntakeArmAngleSpeed(double newIntakeArmAngleSpeed) {
    // Apply arm motor speed immediately.
    intakeArmAngleSpeed = newIntakeArmAngleSpeed;
    intakeMotorArm.set(intakeArmAngleSpeed);
  }

  public void stopArmMotor() {
    // Stop only the arm motor.
    setIntakeArmAngleSpeed(0.0);
  }

  public Command runIntakeCommand(double speed) {
    // Hold a roller speed while the button or command is active.
    return Commands.startEnd(() -> setIntakeMotorSpeed(speed), this::stopIntakeMotor, this);
  }

  public Command runArmCommand(double speed) {
    // Hold an arm movement speed while the button or command is active.
    return Commands.startEnd(() -> setIntakeArmAngleSpeed(speed), this::stopArmMotor, this);
  }

  public Command setIntakeSpeedCommand(double speed) {
    // Latch a roller state for autonomous sequencing.
    return Commands.runOnce(() -> setIntakeMotorSpeed(speed), this);
  }

  public Command setArmSpeedCommand(double speed) {
    // Latch an arm state for autonomous sequencing.
    return Commands.runOnce(() -> setIntakeArmAngleSpeed(speed), this);
  }

  public Command stopIntakeCommand() {
    // One-shot roller stop command.
    return Commands.runOnce(this::stopIntakeMotor, this);
  }

  public Command stopArmCommand() {
    // One-shot arm stop command.
    return Commands.runOnce(this::stopArmMotor, this);
  }
}
