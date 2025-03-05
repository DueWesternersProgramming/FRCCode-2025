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

import java.util.List;
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

    public List<PhotonPipelineResult> getResult() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return photonCamera.getAllUnreadResults();

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
            return getResult().get(0).getBestTarget();
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

    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

            photonPoseEstimator.setLastPose(prevEstimatedRobotPose);
            try {
                List<PhotonPipelineResult> result = photonCamera.getAllUnreadResults(); // Only want to call this once
                                                                                        // per loop.

                if (result.size() > 0) {
                    Optional<EstimatedRobotPose> estimate = photonPoseEstimator
                            .update(result.get(0));

                    if (estimate.isPresent()) {
                        PhotonTrackedTarget furthestTarget = result.get(0).targets.get(0);

                        for (PhotonTrackedTarget target : result.get(0).targets) {
                            if (target.bestCameraToTarget.getTranslation().getNorm() < furthestTarget.bestCameraToTarget
                                    .getTranslation().getNorm()) {
                                furthestTarget = target;
                            }
                        } // Loop through and find the target furthest away, basicly with the most
                          // ambiguity.


                        double smallestTagDistance = furthestTarget.bestCameraToTarget.getTranslation().getNorm();
                        double poseAmbaguitiy = furthestTarget.getPoseAmbiguity();
                        System.out.println(poseAmbaguitiy);
                        if (smallestTagDistance < 3 && poseAmbaguitiy < 5) { // The distance will need to be tuned.
                            //System.out.println(poseAmbaguitiy);
                            
                            return estimate.get();
                        } else {
                            return null;
                        }
                    } else {
                        return null;
                    }
                } else {
                    return null;
                }

            } catch (Exception e) {
                System.out.println(e);
                return null;
            }

        } else

        {
            return null;
        }
    }
}
