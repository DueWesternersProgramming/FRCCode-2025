package frc.robot.subsystems.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.utils.QuestNavUtils;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNavUtils questNav = new QuestNavUtils();

    Transform2d QuestUncorrectedToCorrected = new Transform2d();

    @Override
    public Pose2d getUncorrectedPose() {
        return questNav.getPose();
    }

    @Override
    public Pose2d getCorrectedPose() {
        return questNav.getPose().transformBy(QuestUncorrectedToCorrected);
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        QuestUncorrectedToCorrected = pose.minus(getUncorrectedPose());
    }

    @Override
    public boolean isConnected() {
        return questNav.getConnected();
    }

    @Override
    public void updateInputs(QuestIOInputs inputs) {
        inputs.connected = questNav.getConnected();

        inputs.uncorrectedPose = getUncorrectedPose();
        inputs.correctedPose = getCorrectedPose();

        double timestamp = inputs.timestamp;
        inputs.timestamp = questNav.getTimestamp();
        inputs.timestampDelta = timestamp - inputs.timestamp;
        inputs.batteryLevel = questNav.getBatteryPercent();
        inputs.questUncorrectedToCorrected = QuestUncorrectedToCorrected;

        questNav.processHeartbeat();
        questNav.cleanupResponses();
    }
}