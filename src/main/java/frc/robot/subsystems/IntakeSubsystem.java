package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Intake roller and arm control grouped into one scheduler-owned subsystem. */
public class IntakeSubsystem extends SubsystemBase {
  private static final int kIntakeRollerCurrentLimitAmps = 35;
  private static final int kIntakeArmCurrentLimitAmps = 30;
  private static final double kIntakeRollerRampSeconds = 0.10;
  private static final double kIntakeArmRampSeconds = 0.20;

  // Roller and arm motors for the intake assembly.
  private final SparkFlex intakeMotorArm = new SparkFlex(16, MotorType.kBrushless);
  private final SparkFlex intakeMotor = new SparkFlex(17, MotorType.kBrushless);

  // Cached commanded outputs, useful if the implementation grows to closed-loop control later.
  private double intakeMotorSpeed = 0.0;
  private double intakeArmAngleSpeed = 0.0;

  public IntakeSubsystem() {
    // Roller config favors smooth engagement while the arm config favors resisting backdrive.
    SparkFlexConfig intakeRollerConfig = new SparkFlexConfig();
    intakeRollerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(kIntakeRollerCurrentLimitAmps)
        .openLoopRampRate(kIntakeRollerRampSeconds);

    SparkFlexConfig intakeArmConfig = new SparkFlexConfig();
    intakeArmConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kIntakeArmCurrentLimitAmps)
        .openLoopRampRate(kIntakeArmRampSeconds);

    intakeMotor.configure(
        intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorArm.configure(
        intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

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
