package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
    @AutoLog
    public static class QuestIOInputs {
        public boolean connected = false;

        /** Current transformed ROBOT pose */
        public Pose2d pose = Pose2d.kZero;
        /** Raw QuestNav pose */
        public Pose2d rawPose = Pose2d.kZero;

        public double timestamp = 0;
        public double timestampDelta = 0;
        public double batteryLevel = 0;
    }

    default Pose2d getRobotPose() {
        return new Pose2d();
    }

    default void setRobotPose(Pose2d pose) {
        // Default implementation does nothing
    }

    default boolean isConnected() {
        return false;
    }

    default void updateInputs(QuestIOInputs inputs) {
        inputs.connected = false;
        inputs.pose = Pose2d.kZero;
        inputs.timestamp = 0;
        inputs.timestampDelta = 0;
        inputs.batteryLevel = 0;
    }
}