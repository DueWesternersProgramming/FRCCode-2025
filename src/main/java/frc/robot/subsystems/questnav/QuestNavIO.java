package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
    @AutoLog
    public static class QuestIOInputs {
        public boolean connected = false;

        public Pose2d correctedPose = Pose2d.kZero;
        /** Raw QuestNav pose */
        public Pose2d uncorrectedPose = Pose2d.kZero;

        public double timestamp = 0;
        public double timestampDelta = 0;
        public double batteryLevel = 0;
        public Transform2d questUncorrectedToCorrected = Transform2d.kZero;
    }

    default Pose2d getCorrectedPose() {
        return Pose2d.kZero;
    }

    default Pose2d getUncorrectedPose() {
        return Pose2d.kZero;
    }

    default void setRobotPose(Pose2d pose) {
        // Default implementation does nothing
    }

    default boolean isConnected() {
        return false;
    }

    default void zeroPosition() {
        // Default implementation does nothing
    }

    default void zeroHeading() {
        // Default implementation does nothing
    }

    default void updateInputs(QuestIOInputs inputs) {
        inputs.connected = false;
        inputs.uncorrectedPose = Pose2d.kZero;
        inputs.correctedPose = Pose2d.kZero;
        inputs.timestamp = 0;
        inputs.timestampDelta = 0;
        inputs.batteryLevel = 0;
    }
}