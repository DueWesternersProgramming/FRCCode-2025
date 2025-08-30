package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagCameraIO {
    @AutoLog
    public class AprilTagIOInputs {
        public boolean connected = false;
        public boolean hasTargets = false;
        public int[] targetIDs = new int[0];
        public Pose2d pose = Pose2d.kZero;
        public double bestTargetArea = 0.0;
    }

    default void updateInputs(AprilTagIOInputs inputs) {
    };

    default List<PhotonPipelineResult> getResult() {
        return null;
    }

    default boolean isCameraConnected() {
        return false;
    }

    default PhotonTrackedTarget getBestTarget() {
        return null;
    }

    default double getBestTargetYaw() {
        return 0;
    }

}
