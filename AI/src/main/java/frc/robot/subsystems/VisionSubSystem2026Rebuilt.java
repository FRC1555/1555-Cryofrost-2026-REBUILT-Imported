package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
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
  private PhotonPipelineResult latestResult = new PhotonPipelineResult();
  private Optional<EstimatedRobotPose> latestEstimatedRobotPose = Optional.empty();
  private final List<EstimatedRobotPose> unreadEstimatedRobotPoses = new ArrayList<>();

  public VisionSubSystem2026Rebuilt(String cameraNameR) {
    // Camera name must match the PhotonVision configuration running on the coprocessor.
    cameraR = new PhotonCamera(cameraNameR);

    // Load the official 2026 field tag layout used for pose estimation.
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Camera pose relative to robot center; this must match the real mounting location.
    cameraToRobotR =
        new Transform3d(new Translation3d(0.0, 0.25, 0.6), new Rotation3d(0.0, 0.0, 0.0));

    // Pose estimator converts camera observations into field-relative robot poses.
    photonEstimatorR = new PhotonPoseEstimator(fieldLayout, cameraToRobotR);

    // Publish the Limelight web stream to Shuffleboard for driver and debugging visibility.
    HttpCamera httpCamera = new HttpCamera("FRC1555Camera", "http://limelight-cryo.local:5801");
    CameraServer.startAutomaticCapture(httpCamera);
    Shuffleboard.getTab("Tab").add(httpCamera);
  }

  public boolean hasTargetR() {
    // Fast helper used when other code only needs to know if a target is visible.
    return latestResult.hasTargets();
  }

  public double getTargetYawR() {
    // Return the best target yaw or zero if the frame has no valid targets.
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getYaw();
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
    // Return the newest unread estimate so the drivetrain only consumes each vision measurement once.
    if (unreadEstimatedRobotPoses.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(unreadEstimatedRobotPoses.remove(0));
  }

  public List<EstimatedRobotPose> drainEstimatedRobotPoses() {
    // Batch access is useful when multiple unread frames arrive between drivetrain loops.
    List<EstimatedRobotPose> drainedPoses = new ArrayList<>(unreadEstimatedRobotPoses);
    unreadEstimatedRobotPoses.clear();
    return drainedPoses;
  }

  public Optional<Pose2d> getEstimatedPose2d() {
    // Dashboard helpers should read the latest cached estimate without consuming unread samples.
    return latestEstimatedRobotPose.map(
        est -> {
          var t = est.estimatedPose.getTranslation();
          var r = est.estimatedPose.getRotation();
          return new Pose2d(t.getX(), t.getY(), new Rotation2d(r.getZ()));
        });
  }

  @Override
  public void periodic() {
    // Poll unread frames exactly once per loop and cache the most recent one for dashboard helpers.
    for (PhotonPipelineResult resultR : cameraR.getAllUnreadResults()) {
      latestResult = resultR;

      Optional<EstimatedRobotPose> estimatedRobotPose =
          photonEstimatorR.estimateCoprocMultiTagPose(resultR);
      if (estimatedRobotPose.isEmpty()) {
        estimatedRobotPose = photonEstimatorR.estimateLowestAmbiguityPose(resultR);
      }
      estimatedRobotPose.ifPresent(
          pose -> {
            latestEstimatedRobotPose = Optional.of(pose);
            unreadEstimatedRobotPoses.add(pose);
          });
    }

    SmartDashboard.putBoolean("Has TargetR", latestResult.hasTargets());

    if (latestResult.hasTargets()) {
      // Publish the most useful target and pose-estimation information for tuning.
      PhotonTrackedTarget bestTargetR = latestResult.getBestTarget();

      SmartDashboard.putNumber("Target ID", bestTargetR.getFiducialId());
      SmartDashboard.putNumber("Target Distance", getTargetDistanceR(bestTargetR.getPitch()));
      SmartDashboard.putNumber("Number of TargetR", latestResult.getTargets().size());
      getEstimatedPose2d()
          .ifPresent(
              estimatedPose -> SmartDashboard.putString("EstimatedPose", estimatedPose.toString()));
    } else {
      // Clear target-dependent dashboard values when the camera loses sight of tags.
      SmartDashboard.putNumber("Target IDR", -1);
      SmartDashboard.putNumber("Number of TargetsR", 0);
      SmartDashboard.putString("EstimatedPose", "none");
    }
  }
}
