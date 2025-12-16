package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants.QuestNavConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNav questNav = new QuestNav();

    @Override
    public Pose3d getUncorrectedPose() {
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
        if (poseFrames.length > 0) {
            // Get the most recent Quest relative pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
            return questPose;
        }
        return Pose3d.kZero;
    }

    @Override
    public Pose3d getCorrectedPose() {
        return getUncorrectedPose().transformBy(new Transform3d(QuestNavConstants.ROBOT_TO_QUEST.inverse()));
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose.transformBy(QuestNavConstants.ROBOT_TO_QUEST)));
    }

    @Override
    public boolean isConnected() {
        return questNav.isConnected();
    }

    @Override
    public void updateInputs(QuestIOInputs inputs) {

        inputs.connected = isConnected();

        inputs.uncorrectedPose = getUncorrectedPose();
        inputs.correctedPose = getCorrectedPose();

        double timestamp = inputs.timestamp;
        inputs.timestamp = questNav.getAppTimestamp().getAsDouble();
        inputs.timestampDelta = timestamp - inputs.timestamp;
        inputs.batteryLevel = questNav.getBatteryPercent().getAsInt();

        questNav.commandPeriodic();

    }
}
