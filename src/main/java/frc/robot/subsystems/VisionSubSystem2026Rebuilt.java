package frc.robot.subsystems;

import java.util.Optional;
//import java.nio.file.Path;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class VisionSubSystem2026Rebuilt extends SubsystemBase {
    private final PhotonCamera cameraR;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraToRobotR;

    public VisionSubSystem2026Rebuilt(String cameraNameR) {
            // Initialize the PhotonCamera with the name of your camera
        cameraR = new PhotonCamera("RightCAM");

        // Load the AprilTag field layout for the 2026 Rebuilt Andymark field
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // Define the transformation from the camera to the robot's center
        cameraToRobotR = new Transform3d(
            new Translation3d(0.0, 0.25, 0.6), // x, y, z offsets in meters
            new Rotation3d(0.0, 0.0, 0.0) ); // roll, pitch, yaw in radians

        // Initialize the pose estimator after fieldLayout and cameraToRobotR are set
        // photonEstimatorR = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cameraToRobotR);
    }

    

    //Right Target Camera
    public boolean hasTargetR() {
        // Check if the camera sees a target
        PhotonPipelineResult resultR = cameraR.getLatestResult();
        return resultR.hasTargets();
    }
    

        public double getTargetYawR() {
        // Get the yaw (horizontal angle) of the best target
        PhotonPipelineResult resultR = cameraR.getLatestResult();
        if (resultR.hasTargets()) {
            return resultR.getBestTarget().getYaw();
        }
        return 0.0; // Default value if no target is found
    }

         //Right Camera Pitch
     public double getTargetDistanceR(double pitchR) {
         // Calculate distance based on the given target pitch (requires calibration)
         // Replace with your own distance calculation formula
         return calculateDistanceFromPitchR(pitchR);
    }
         //Right Camera Pitch Math
     private double calculateDistanceFromPitchR(double pitchR) {
          //Example formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + pitch)
         double targetHeightR = 1.124; // Replace with the height of your target in meters
         double cameraHeightR = 0.6; // Replace with the height of your camera in meters
         double cameraAngleR = Math.toRadians(0.79); // Replace with your camera's mounting angle in radians
         return (targetHeightR - cameraHeightR) / Math.tan(cameraAngleR + Math.toRadians(pitchR));
    }

        public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    //public Transform3d getCameraToRobotR() {
    //private final PhotonPoseEstimator photonEstimatorR;
   // }

    PhotonPoseEstimator photonEstimatorR = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
        PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
        new Transform3d(
            new Translation3d(0.0, 0.25, 0.6), // x, y, z offsets in meters
            new Rotation3d(0.0, 0.0, 0.0) ));
    
        @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult resultR = cameraR.getLatestResult();

        // Publish whether a target is detected
        SmartDashboard.putBoolean("Has TargetR", resultR.hasTargets());


        //Camera R Data
        if (resultR.hasTargets()) {
            PhotonTrackedTarget bestTargetR = resultR.getBestTarget();

            // Publish data about the best target
            SmartDashboard.putNumber("Target YawR", bestTargetR.getYaw());
            SmartDashboard.putNumber("Target PitchR", bestTargetR.getPitch());
            SmartDashboard.putNumber("Target AreaR", bestTargetR.getArea());
            SmartDashboard.putNumber("Target SkewR", bestTargetR.getSkew());
            SmartDashboard.putNumber("Target IDR", bestTargetR.getFiducialId());
            SmartDashboard.putNumber("Target DistanceR", getTargetDistanceR(bestTargetR.getPitch()));

        photonEstimatorR.estimateLowestAmbiguityPose(resultR).ifPresentOrElse(
        est -> SmartDashboard.putString("EstimatedPose", est.estimatedPose.toString()),
        () -> SmartDashboard.putString("EstimatedPose", "none")
        );
            // SmartDashboard.putNumber("TransForm3DR(X)", getCameraToRobotR().getTranslation().getX());
            // SmartDashboard.putNumber("TransForm3DR(Y)", getCameraToRobotR().getTranslation().getY());
            // SmartDashboard.putNumber("TransForm3DR(Z)", getCameraToRobotR().getTranslation().getZ()); // Example: publish Z component of transform


            // Publish the number of targets detected
            SmartDashboard.putNumber("Number of TargetR", resultR.getTargets().size());
        } else {
            // Clear the data if no target is detected
            SmartDashboard.putNumber("Target YawR", 0.0);
            SmartDashboard.putNumber("Target PitchR", 0.0);
            SmartDashboard.putNumber("Target AreaR", 0.0);
            SmartDashboard.putNumber("Target SkewR", 0.0);
            SmartDashboard.putNumber("Target IDR", -1);
            SmartDashboard.putNumber("Number of TargetsR", 0);
        }
}
}

