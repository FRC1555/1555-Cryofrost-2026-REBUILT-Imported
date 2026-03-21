package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Single active shooter motor used by both teleop and autonomous routines.
  private final SparkFlex shooterMotorRight = new SparkFlex(19, MotorType.kBrushless);
  private double shooterMotorRightSpeed = 0.0;

  public void setShooterMotorSpeed(double newShooterMotorSpeed) {
    // Update the stored target and apply it immediately to hardware.
    shooterMotorRightSpeed = newShooterMotorSpeed;
    shooterMotorRight.set(shooterMotorRightSpeed);
  }

  public void stopShooter() {
    // Shared stop helper for button release and auto cleanup.
    setShooterMotorSpeed(0.0);
  }

  public Command runShooterCommand(double speed) {
    // Hold the requested shooter speed only while the triggering command remains scheduled.
    return Commands.startEnd(() -> setShooterMotorSpeed(speed), this::stopShooter, this);
  }

  public Command setShooterSpeedCommand(double speed) {
    // Latch a shooter speed until another command changes it.
    return Commands.runOnce(() -> setShooterMotorSpeed(speed), this);
  }

  public Command stopShooterCommand() {
    // Explicit stop command used in autos and controller release handlers.
    return Commands.runOnce(this::stopShooter, this);
  }
}
