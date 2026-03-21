package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubSystem2026Rebuilt extends SubsystemBase {
  // PhotonVision camera and its associated field/robot pose model.
  private final PhotonCamera cameraR;
  private final AprilTagFieldLayout fieldLayout;
  private final Transform3d cameraToRobotR;
  private final PhotonPoseEstimator photonEstimatorR;

  public VisionSubSystem2026Rebuilt(String cameraNameR) {
    // Camera name must match the PhotonVision configuration running on the coprocessor.
    cameraR = new PhotonCamera(cameraNameR);

    // Load the official 2026 field tag layout used for pose estimation.
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Camera pose relative to robot center; this must match the real mounting location.
    cameraToRobotR =
        new Transform3d(new Translation3d(0.0, 0.25, 0.6), new Rotation3d(0.0, 0.0, 0.0));

    // Pose estimator converts camera observations into field-relative robot poses.
    photonEstimatorR =
        new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cameraToRobotR);

    // Publish the Limelight web stream to Shuffleboard for driver and debugging visibility.
    HttpCamera httpCamera = new HttpCamera("FRC1555Camera", "http://limelight-cryo.local:5801");
    CameraServer.startAutomaticCapture(httpCamera);
    Shuffleboard.getTab("Tab").add(httpCamera);
  }

  public boolean hasTargetR() {
    // Fast helper used when other code only needs to know if a target is visible.
    return cameraR.getLatestResult().hasTargets();
  }

  public double getTargetYawR() {
    // Return the best target yaw or zero if the frame has no valid targets.
    PhotonPipelineResult resultR = cameraR.getLatestResult();
    if (resultR.hasTargets()) {
      return resultR.getBestTarget().getYaw();
    }
    return 0.0;
  }

  public double getTargetDistanceR(double pitchR) {
    // Wrapper kept so call sites do not need to know the underlying pitch-distance math.
    return calculateDistanceFromPitchR(pitchR);
  }

  private double calculateDistanceFromPitchR(double pitchR) {
    // Basic trig estimate from camera mounting geometry to target plane.
    double targetHeightR = 1.124;
    double cameraHeightR = 0.6;
    double cameraAngleR = Math.toRadians(0.79);
    return (targetHeightR - cameraHeightR) / Math.tan(cameraAngleR + Math.toRadians(pitchR));
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    // PhotonVision returns a stamped estimate so the drivetrain can fuse it with the correct latency.
    PhotonPipelineResult resultR = cameraR.getLatestResult();
    return photonEstimatorR.estimateLowestAmbiguityPose(resultR);
  }

  public Optional<Pose2d> getEstimatedPose2d() {
    // Convenience wrapper for code that only needs the 2D field pose.
    return getEstimatedRobotPose().map(
        est -> {
          var t = est.estimatedPose.getTranslation();
          var r = est.estimatedPose.getRotation();
          return new Pose2d(t.getX(), t.getY(), new Rotation2d(r.getZ()));
        });
  }

  @Override
  public void periodic() {
    // Poll the latest camera frame once per scheduler loop.
    PhotonPipelineResult resultR = cameraR.getLatestResult();

    SmartDashboard.putBoolean("Has TargetR", resultR.hasTargets());

    if (resultR.hasTargets()) {
      // Publish the most useful target and pose-estimation information for tuning.
      PhotonTrackedTarget bestTargetR = resultR.getBestTarget();

      SmartDashboard.putNumber("Target ID", bestTargetR.getFiducialId());
      SmartDashboard.putNumber("Target Distance", getTargetDistanceR(bestTargetR.getPitch()));
      SmartDashboard.putNumber("Number of TargetR", resultR.getTargets().size());
      getEstimatedRobotPose()
          .ifPresent(
              estimatedRobotPose ->
                  SmartDashboard.putString(
                      "EstimatedPose", estimatedRobotPose.estimatedPose.toPose2d().toString()));
    } else {
      // Clear target-dependent dashboard values when the camera loses sight of tags.
      SmartDashboard.putNumber("Target IDR", -1);
      SmartDashboard.putNumber("Number of TargetsR", 0);
      SmartDashboard.putString("EstimatedPose", "none");
    }
  }
}
