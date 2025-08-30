package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.RobotState;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.RobotConstants.VisionConstants.VisionSource;
import frc.robot.utils.CowboyUtils;

import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagCameraIOPhotonSim extends AprilTagCameraIOPhoton {
    private final PhotonCameraSim cameraSim;

    public AprilTagCameraIOPhotonSim(
            VisionSource source,
            SimCameraConfig config) {
        super(source); // Explicitly call the superclass constructor

        SimCameraProperties props = config.apply(new SimCameraProperties());

        cameraSim = new PhotonCameraSim(photonCamera, props, CowboyUtils.aprilTagFieldLayout);

        cameraSim.enableDrawWireframe(true);
        cameraSim.setMaxSightRange(10.0);
        cameraSim.setWireframeResolution(1);

        VisionConstants.aprilTagSim.ifPresent(
                aprilTagSim -> aprilTagSim.addCamera(cameraSim, source.robotToCamera()));
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        super.updateInputs(inputs);

        VisionConstants.aprilTagSim.ifPresent(
                aprilTagSim -> {
                    FieldObject2d visionEstimation = aprilTagSim.getDebugField().getObject("VisionEstimation");

                    visionEstimation.setPoses(visionEstimation.getPoses());
                });
    }

    // TODO

    // @Override
    // public List<PhotonPipelineResult> getResult() {
    // if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
    // return photonCamera.getAllUnreadResults();

    // } else {
    // return null;
    // }
    // }
}