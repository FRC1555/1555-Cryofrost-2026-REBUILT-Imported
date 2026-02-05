package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionWithHelp {
    PhotonCamera camera = new PhotonCamera("YOUR_CAMERA_NAME");
PhotonPoseEstimator estimator = new PhotonPoseEstimator(
    AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndymark),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    camera,
    new Transform3d(/* your robot-to-camera transform */)
);

// In periodic / command
var result = estimator.update();
if (result.isPresent()) {
    Pose3d robotPose = result.get().estimatedPose.toPose3d();
    double timestamp = result.get().timestampSeconds;

    System.out.println("Estimated pose: " + robotPose);
    // Feed to SwerveDrivePoseEstimator here
}
}
