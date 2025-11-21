package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
    @AutoLog
    public static class QuestIOInputs {
        public boolean connected = false;

        public Pose3d correctedPose = Pose3d.kZero;
        /** Raw QuestNav pose */
        public Pose3d uncorrectedPose = Pose3d.kZero;

        public double timestamp = 0;
        public double timestampDelta = 0;
        public double batteryLevel = 0;
    }

    default Pose3d getCorrectedPose() {
        return Pose3d.kZero;
    }

    default Pose3d getUncorrectedPose() {
        return Pose3d.kZero;
    }

    default void setRobotPose(Pose2d pose) {
        // Default implementation does nothing
    }

    default boolean isConnected() {
        return false;
    }

    default void updateInputs(QuestIOInputs inputs) {
        inputs.connected = false;
        inputs.uncorrectedPose = Pose3d.kZero;
        inputs.correctedPose = Pose3d.kZero;
        inputs.timestamp = 0;
        inputs.timestampDelta = 0;
        inputs.batteryLevel = 0;
    }
}