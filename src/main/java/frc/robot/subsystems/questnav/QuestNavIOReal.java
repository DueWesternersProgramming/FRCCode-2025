package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNavUtils questNav = new QuestNavUtils();

    // First, Declare our geometrical transform from the Quest to the robot center
    Transform2d QUEST_TO_ROBOT = new Transform2d();

    @Override
    public Pose2d getQuestPose() {
        return questNav.getPose();
    }

    @Override
    public Pose2d getRobotPose() {
        return questNav.getPose().transformBy(QUEST_TO_ROBOT.inverse());
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        questNav.setPose(pose.transformBy(QUEST_TO_ROBOT));
    }

    @Override
    public boolean isConnected() {
        return questNav.getConnected();
    }

    @Override
    public void updateInputs(QuestIOInputs inputs) {
        inputs.connected = questNav.getConnected();

        inputs.rawPose = questNav.getPose();
        inputs.pose = getRobotPose();

        double timestamp = inputs.timestamp;
        inputs.timestamp = questNav.getTimestamp();
        inputs.timestampDelta = timestamp - inputs.timestamp;
        inputs.batteryLevel = questNav.getBatteryPercent();

        questNav.processHeartbeat();
        questNav.cleanupResponses();
    }
}