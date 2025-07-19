package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagCameraIO {
    @AutoLog
    public class AprilTagIOInputs {
        public boolean connected = false;
        public boolean hasTargets = false;
        public int[] targetIDs = new int[0];
        public Pose2d pose = Pose2d.kZero;
    }

    default void updateInputs(AprilTagIOInputs inputs) {
    };

}
