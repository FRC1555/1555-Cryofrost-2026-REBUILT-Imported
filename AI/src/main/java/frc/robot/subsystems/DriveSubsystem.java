// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/** Swerve drivetrain with PathPlanner integration and fused AprilTag vision odometry. */
public class DriveSubsystem extends SubsystemBase {
  // The robot uses one IMU axis consistently for driver heading, odometry, and auto control.
  private static final IMUAxis kYawAxis = IMUAxis.kZ;

  // Individual swerve modules at each corner of the chassis.
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // Shared sensors and estimator inputs.
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final VisionSubSystem2026Rebuilt m_visionSubsystem;

  // Last commanded chassis speeds, primarily useful for tuning and dashboard work later.
  private double xSpeedDelivered;
  private double ySpeedDelivered;
  private double rotDelivered;

  // Driver-adjustable top-speed scaling.
  public double currentDriveSpeed = 1.0;

  // Fused pose estimator combining wheel odometry with asynchronous vision measurements.
  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          getYaw(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d());

  public DriveSubsystem(VisionSubSystem2026Rebuilt visionSubsystem) {
    m_visionSubsystem = visionSubsystem;

    // Trust drivetrain odometry for heading and allow moderate XY correction from vision.
    m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // PathPlanner hooks into the drivetrain here for pose reset, motion commands, and alliance mirroring.
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false),
        this);
  }

  private Rotation2d getYaw() {
    // All heading-based calculations read the same IMU axis to avoid mismatched frame math.
    return Rotation2d.fromDegrees(m_gyro.getAngle(kYawAxis));
  }

  public void updateOdometryWithVision(Pose2d visionPose, double timestamp) {
    // Utility hook if another vision source needs to inject a stamped pose later.
    m_odometry.addVisionMeasurement(visionPose, timestamp);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    // PathPlanner supplies robot-relative chassis speeds during auto tracking.
    setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void resetPose(Pose2d pose) {
    // Full estimator reset used when we want to replace both translation and rotation at once.
    m_odometry.resetPosition(
        getYaw(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    // Convert module states back into chassis-relative motion for PathPlanner feedback.
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  @Override
  public void periodic() {
    // First update wheel/gyro odometry every scheduler cycle.
    m_odometry.update(
        getYaw(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // Then fuse the most recent stamped vision pose if PhotonVision has one available.
    for (var estimatedRobotPose : m_visionSubsystem.drainEstimatedRobotPoses()) {
      m_odometry.addVisionMeasurement(
          estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
    }
  }

  public Pose2d getPose() {
    // Current fused field pose used by auto and dashboard tooling.
    return m_odometry.getEstimatedPosition();
  }

  public void zeroGyro() {
    // Preserve translation while redefining the robot's current heading as zero.
    resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
  }

  public void resetOdometry(Pose2d pose) {
    // Standard PathPlanner reset entry point for autos with known starting poses.
    m_odometry.resetPosition(
        getYaw(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  public void setDriveSpeed(double newDriveSpeed) {
    // Driver speed scaling stays separate from the physical max speed constants.
    currentDriveSpeed = newDriveSpeed;
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Scale normalized joystick inputs into real chassis speed requests.
    xSpeedDelivered = -xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * currentDriveSpeed;
    ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * currentDriveSpeed;
    rotDelivered = rot * DriveConstants.kMaxAngularSpeed * currentDriveSpeed;

    // Convert into swerve module states, optionally using the field frame for driver control.
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getYaw())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command setXCommand() {
    // X-lock is useful for resisting pushes while stopped.
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Shared helper for manual drive, auto drive, and any future closed-loop routines.
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    // Zero all drive encoders when troubleshooting or re-homing the drivetrain.
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public double getHeading() {
    // Public heading accessor in degrees for dashboards or helper utilities.
    return getYaw().getDegrees();
  }

  public double getTurnRate() {
    // Use the same yaw axis as the heading calculation.
    return m_gyro.getRate(kYawAxis) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetGyro() {
    // Reset the yaw axis only; other axes remain untouched.
    m_gyro.setGyroAngle(kYawAxis, 0);
  }
}
