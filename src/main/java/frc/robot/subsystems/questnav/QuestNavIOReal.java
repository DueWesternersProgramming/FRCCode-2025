package frc.robot.subsystems.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants.QuestNavConstants;
import frc.robot.utils.QuestNavUtils;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNavUtils questNav = new QuestNavUtils();

    // Only store translational offset
    private Translation2d translationOffset = new Translation2d();

    @Override
    public Pose2d getUncorrectedPose() {
        return questNav.getUncorrectedPose();
    }

    @Override
    public Pose2d getCorrectedPose() {
        Translation2d translated = getUncorrectedPose().getTranslation().plus(translationOffset);

        Pose2d poseAtQuest = new Pose2d(translated, questNav.getUncorrectedYaw());

        Pose2d robotCenterPose = poseAtQuest.transformBy(QuestNavConstants.questToRobotCenter);

        return robotCenterPose;
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        Translation2d currentUncorrected = getUncorrectedPose().getTranslation();
        translationOffset = pose.getTranslation().minus(currentUncorrected);
    }

    @Override
    public boolean isConnected() {
        return questNav.connected();
    }

    @Override
    public void zeroPosition() {
        questNav.zeroPosition();
    }

    @Override
    public void zeroHeading() {
        questNav.zeroHeading();
    }

    @Override
    public void updateInputs(QuestIOInputs inputs) {
        inputs.connected = isConnected();

        inputs.uncorrectedPose = getUncorrectedPose();
        inputs.correctedPose = getCorrectedPose();

        double timestamp = inputs.timestamp;
        inputs.timestamp = questNav.timestamp();
        inputs.timestampDelta = timestamp - inputs.timestamp;
        inputs.batteryLevel = questNav.getBatteryPercent();
        inputs.questUncorrectedToCorrected = new edu.wpi.first.math.geometry.Transform2d(
                translationOffset, Rotation2d.kZero);

        questNav.processHeartbeat();
        questNav.cleanUpQuestNavMessages();
    }
}
