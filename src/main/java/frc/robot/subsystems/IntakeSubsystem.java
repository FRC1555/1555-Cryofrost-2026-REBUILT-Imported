package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Floor intake subsystem (note pickup).
 *
 * <p>CAN IDs:
 * <ul>
 *   <li>16 - intakeMotorArm: controls up/down pivot of the intake mechanism
 *   <li>17 - intakeMotor: controls the roller wheels that pull the note in
 * </ul>
 *
 * <p>Set positive intakeArm speed to move arm down, negative to move up (actual direction depends
 * on mechanism geometry).
 */
public class IntakeSubsystem {

  public final static SparkFlex intakeMotorArm = new SparkFlex(16, MotorType.kBrushless);
  public final static SparkFlex intakeMotor = new SparkFlex(17, MotorType.kBrushless);

  public static double intakeMotorSpeed = 0.02;
  public static double IntakeArmAngleSpeed = 0.02;

  public void IntakeSystem() {
    intakeMotor.set(intakeMotorSpeed);
  }

  public void setintakeMotorSpeed(double newIntakeMotorSpeed) {
    intakeMotorSpeed = newIntakeMotorSpeed;
  }

  public void IntakeArmSystem() {
    intakeMotorArm.set(IntakeArmAngleSpeed);
  }
}
