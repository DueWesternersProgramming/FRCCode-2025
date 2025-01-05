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
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70)); // Make these constants
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update(new PhotonPipelineResult());
        } else {
            return null;
        }
    }
}
