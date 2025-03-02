package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.utils.CowboyUtils;
import java.util.Optional;

public class Camera {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    public Camera(String cameraName, Transform3d positionTransform3d) {
        photonCamera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                CowboyUtils.aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_LAST_POSE,
                positionTransform3d);

        // photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setPipeline(int index) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            photonCamera.setPipelineIndex(index);
        }
    }

    public PhotonPipelineResult getResult() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return photonCamera.getLatestResult();

        } else {
            return null;
        }
    }

    public boolean isCameraConnected() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return photonCamera.isConnected();
        } else {
            return false;
        }
    }

    public boolean hasResults() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return photonCamera.getLatestResult().hasTargets();
        } else {
            return false;
        }
    }

    public PhotonTrackedTarget getBestTarget() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return getResult().getBestTarget();
        } else {
            return null;
        }
    }

    public double getTargetYaw() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return getBestTarget().getYaw();
        } else {
            return 0;
        }
    }

    public Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

            photonPoseEstimator.setLastPose(prevEstimatedRobotPose);
            try {

                // System.out.println((photonCamera.getLatestResult().getBestTarget().fiducialId));

                Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update(photonCamera.getLatestResult());
                // System.out.println(estimate.isPresent());
                return estimate.isPresent() ? estimate.get().estimatedPose.toPose2d() : null;

            } catch (Exception e) {
                System.out.println(e);
                return null;
            }

        } else {
            return null;
        }
    }
}
