package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Shooter control with encoder-based velocity regulation for more repeatable shots. */
@SuppressWarnings("removal")
public class ShooterSubsystem extends SubsystemBase {
  private static final double kShooterFreeSpeedRpm = Constants.NeoMotorConstants.kFreeSpeedRpm;
  private static final double kShooterVelocityP = 0.00025;
  private static final double kShooterVelocityI = 0.0;
  private static final double kShooterVelocityD = 0.0;
  private static final double kShooterVelocityFF = 1.0 / kShooterFreeSpeedRpm;
  private static final int kShooterCurrentLimitAmps = 50;
  private static final double kShooterVoltageComp = 12.0;
  private static final double kShooterReadyToleranceRpm = 150.0;
  private static final double kShooterReadyDebounceSeconds = 0.15;
  private static final double kMinimumReadyTargetRpm = 500.0;

  public static final SparkFlex ShooterMotorRight = new SparkFlex(19, MotorType.kBrushless);

  private final RelativeEncoder shooterEncoder = ShooterMotorRight.getEncoder();
  private final SparkClosedLoopController shooterController =
      ShooterMotorRight.getClosedLoopController();
  private final Debouncer shooterReadyDebouncer = new Debouncer(kShooterReadyDebounceSeconds);

  private double shooterPercentSetpoint = 0.0;
  private double shooterVelocitySetpointRpm = 0.0;

  public ShooterSubsystem() {
    SparkFlexConfig shooterConfig = new SparkFlexConfig();
    shooterConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(kShooterCurrentLimitAmps)
        .voltageCompensation(kShooterVoltageComp);
    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kShooterVelocityP, kShooterVelocityI, kShooterVelocityD)
        .velocityFF(kShooterVelocityFF)
        .outputRange(-1.0, 1.0);

    ShooterMotorRight.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private static double clampPercent(double requestedSpeed) {
    return Math.max(-1.0, Math.min(1.0, requestedSpeed));
  }

  private static double percentToRpm(double shooterPercent) {
    return clampPercent(shooterPercent) * kShooterFreeSpeedRpm;
  }

  public void setShooterMotorSpeed(double newShooterMotorSpeed) {
    shooterPercentSetpoint = clampPercent(newShooterMotorSpeed);
    shooterVelocitySetpointRpm = percentToRpm(shooterPercentSetpoint);
    shooterController.setReference(shooterVelocitySetpointRpm, ControlType.kVelocity);
  }

  public void stopShooter() {
    shooterPercentSetpoint = 0.0;
    shooterVelocitySetpointRpm = 0.0;
    ShooterMotorRight.stopMotor();
  }

  public double getShooterVelocityRpm() {
    return shooterEncoder.getVelocity();
  }

  public double getShooterVelocitySetpointRpm() {
    return shooterVelocitySetpointRpm;
  }

  public double getShooterVelocityErrorRpm() {
    return shooterVelocitySetpointRpm - getShooterVelocityRpm();
  }

  public boolean isAtSpeed() {
    return shooterVelocitySetpointRpm > kMinimumReadyTargetRpm
        && Math.abs(getShooterVelocityErrorRpm()) < kShooterReadyToleranceRpm;
  }

  public boolean isReadyToFire() {
    return shooterReadyDebouncer.calculate(isAtSpeed());
  }

  public void periodic() {
    boolean atSpeed = isAtSpeed();
    boolean readyToFire = isReadyToFire();

    SmartDashboard.putNumber("Shooter/VelocityRPM", getShooterVelocityRpm());
    SmartDashboard.putNumber("Shooter/TargetRPM", shooterVelocitySetpointRpm);
    SmartDashboard.putNumber("Shooter/VelocityErrorRPM", getShooterVelocityErrorRpm());
    SmartDashboard.putNumber("Shooter/TargetPercent", shooterPercentSetpoint);
    SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed);
    SmartDashboard.putBoolean("Shooter/ReadyToFire", readyToFire);
  }
}
