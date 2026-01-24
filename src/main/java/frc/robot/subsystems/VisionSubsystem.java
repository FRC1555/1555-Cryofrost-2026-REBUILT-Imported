package frc.robot.subsystems;

import java.util.Optional;
//import java.nio.file.Path;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraR;
    private final PhotonCamera cameraL;

    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraToRobot;

    public VisionSubsystem(String cameraName) {
        // Initialize the PhotonCamera with the name of your camera
        cameraR = new PhotonCamera("RightCAM");
        cameraL = new PhotonCamera("LeftCam");


        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        // Load the AprilTagFieldLayout for the 2023 field
       // fieldLayout = AprilTagFieldLayout(relativePath).orElseThrow(
         //   () -> new IllegalStateException("Failed to load AprilTagFieldLayout")
       // );

        // Define the transformation from the camera to the robot's center
        cameraToRobot = new Transform3d(
            new Translation3d(0.3556, 0.0762, 0.3556), // x, y, z offsets in meters
            new Rotation3d(0.0, 0.0, 0.0)    // roll, pitch, yaw in radians
        );
    }

    //Right Target Camera
    public boolean hasTargetR() {
        // Check if the camera sees a target
        PhotonPipelineResult resultR = cameraR.getLatestResult();
        return resultR.hasTargets();
    }

    //Left Target Camera
    public boolean hasTargetL() {
        // Check if the camera sees a target
        PhotonPipelineResult resultL = cameraL.getLatestResult();
        return resultL.hasTargets();
    }
    

    //Camera R Target Yaw
    public double getTargetYawR() {
        // Get the yaw (horizontal angle) of the best target
        PhotonPipelineResult resultR = cameraR.getLatestResult();
        if (resultR.hasTargets()) {
            return resultR.getBestTarget().getYaw();
        }
        return 0.0; // Default value if no target is found
    }

    //Camera L Target Yaw
    public double getTargetYawL() {
        // Get the yaw (horizontal angle) of the best target
        PhotonPipelineResult resultL = cameraL.getLatestResult();
        if (resultL.hasTargets()) {
            return resultL.getBestTarget().getYaw();
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
        // Example formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + pitch)
        double targetHeightR = 2.5; // Replace with the height of your target in meters
        double cameraHeightR = 1.0; // Replace with the height of your camera in meters
        double cameraAngleR = Math.toRadians(30); // Replace with your camera's mounting angle in radians
        return (targetHeightR - cameraHeightR) / Math.tan(cameraAngleR + Math.toRadians(pitchR));
    }

    //Left Camera Pitch
    public double getTargetDistanceL(double pitchL) {
        // Calculate distance based on the given target pitch (requires calibration)
        // Replace with your own distance calculation formula
        return calculateDistanceFromPitchL(pitchL);
    }

    //Left Camera Pitch Math
    private double calculateDistanceFromPitchL(double pitchL) {
        // Example formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + pitch)
        double targetHeightL = 2.5; // Replace with the height of your target in meters
        double cameraHeightL = 1.0; // Replace with the height of your camera in meters
        double cameraAngleL = Math.toRadians(30); // Replace with your camera's mounting angle in radians
        return (targetHeightL - cameraHeightL) / Math.tan(cameraAngleL + Math.toRadians(pitchL));
    }


        public Optional<Pose2d> getEstimatedPose() {
        PhotonPipelineResult resultR = cameraR.getLatestResult();
    
        if (resultR.hasTargets()) {
            PhotonTrackedTarget bestTarget = resultR.getBestTarget();
            double distanceR = getTargetDistanceR(bestTarget.getPitch());
            double yawR = bestTarget.getYaw();
    
            // Calculate the robot's position relative to the field
            Pose2d estimatedPoseR = new Pose2d(
                /* x */ distanceR * Math.cos(Math.toRadians(yawR)),
                /* y */ distanceR * Math.sin(Math.toRadians(yawR)),
                /* rotation */ new Rotation2d(Math.toRadians(yawR))
            );
            return Optional.of(estimatedPoseR);
        }
        return Optional.empty();
    } 

    
    public Optional<Pose2d> getEstimatedPoseL() {
        PhotonPipelineResult resultL = cameraL.getLatestResult();
    
        if (resultL.hasTargets()) {
            PhotonTrackedTarget bestTarget = resultL.getBestTarget();
            double distanceL = getTargetDistanceR(bestTarget.getPitch());
            double yawL = bestTarget.getYaw();
    
            // Calculate the robot's position relative to the field
            Pose2d estimatedPoseL = new Pose2d(
                /* x */ distanceL * Math.cos(Math.toRadians(yawL)),
                /* y */ distanceL * Math.sin(Math.toRadians(yawL)),
                /* rotation */ new Rotation2d(Math.toRadians(yawL))
            );
            return Optional.of(estimatedPoseL);
        }
        return Optional.empty();
    
    }

    

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public Transform3d getCameraToRobot() {
        return cameraToRobot;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult resultR = cameraR.getLatestResult();
        PhotonPipelineResult resultL = cameraL.getLatestResult();

        // Publish whether a target is detected
        SmartDashboard.putBoolean("Has TargetR", resultR.hasTargets());
        SmartDashboard.putBoolean("Has TargetL", resultL.hasTargets());

        //Camera R Data
        if (resultR.hasTargets()) {
            PhotonTrackedTarget bestTargetR = resultR.getBestTarget();

            // Publish data about the best target
            SmartDashboard.putNumber("Target YawR", bestTargetR.getYaw());
            SmartDashboard.putNumber("Target PitchR", bestTargetR.getPitch());
            SmartDashboard.putNumber("Target AreaR", bestTargetR.getArea());
            SmartDashboard.putNumber("Target SkewR", bestTargetR.getSkew());
            SmartDashboard.putNumber("Target IDR", bestTargetR.getFiducialId());

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

        //Camera L Data
        if (resultL.hasTargets()) {
            PhotonTrackedTarget bestTargetL = resultL.getBestTarget();

            // Publish data about the best target
            SmartDashboard.putNumber("Target YawL", bestTargetL.getYaw());
            SmartDashboard.putNumber("Target PitchL", bestTargetL.getPitch());
            SmartDashboard.putNumber("Target AreaL", bestTargetL.getArea());
            SmartDashboard.putNumber("Target SkewL", bestTargetL.getSkew());
            SmartDashboard.putNumber("Target IDL", bestTargetL.getFiducialId());

            // Publish the number of targets detected
            SmartDashboard.putNumber("Number of TargetsL", resultL.getTargets().size());
        } else {
            // Clear the data if no target is detected
            SmartDashboard.putNumber("Target YawL", 0.0);
            SmartDashboard.putNumber("Target PitchL", 0.0);
            SmartDashboard.putNumber("Target AreaL", 0.0);
            SmartDashboard.putNumber("Target SkewL", 0.0);
            SmartDashboard.putNumber("Target IDL", -1);
            SmartDashboard.putNumber("Number of TargetsL", 0);
        }
    }
    
}

