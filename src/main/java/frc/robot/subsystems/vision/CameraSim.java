package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.utils.CowboyUtils;

import java.util.List;
import java.util.Optional;

public class CameraSim {
    private SimCameraProperties cameraProp;
    public PhotonCameraSim photonCameraSim;
    private PhotonPoseEstimator photonPoseEstimator;
    public PhotonCamera camera;

    public CameraSim(String cameraName, Transform3d positionTransform3d) {
        camera = new PhotonCamera(cameraName);
        cameraProp = new SimCameraProperties();
        setCameraProperties();
        photonCameraSim = new PhotonCameraSim(camera, cameraProp);
        photonCameraSim.enableDrawWireframe(true);
        photonCameraSim.setMaxSightRange(10.0);
        photonCameraSim.setWireframeResolution(0.25);

        photonPoseEstimator = new PhotonPoseEstimator(
                CowboyUtils.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                positionTransform3d);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    private void setCameraProperties() {
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70)); // Make these constants
    }

    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

            photonPoseEstimator.setLastPose(prevEstimatedRobotPose);
            try {
                List<PhotonPipelineResult> result = camera.getAllUnreadResults(); // Only want to call this once
                                                                                  // per loop, so we do it at the start.

                if (result.size() > 0) {
                    Optional<EstimatedRobotPose> estimate = photonPoseEstimator
                            .update(result.get(0));

                    if (estimate.isPresent()) {
                        double smallestTagDistance = result.get(0).getBestTarget().bestCameraToTarget.getTranslation()
                                .getNorm();
                        double poseAmbaguitiy = result.get(0).getBestTarget().getPoseAmbiguity();
                        if (smallestTagDistance < 5 && poseAmbaguitiy < 0.05) { // The distance will need to be tuned.
                            return estimate.get();
                        }
                        return null;// No need for else statment
                    }
                    return null; // No need for else statment

                }
                return null; // No need for else statment

            } catch (Exception e) {
                System.out.println(e);
                return null;
            }

        } else {
            return null;
        }
    }
}
