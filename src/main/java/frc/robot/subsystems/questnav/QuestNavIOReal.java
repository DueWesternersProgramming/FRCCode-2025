package frc.robot.subsystems.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants.QuestNavConstants;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNav questNav = new QuestNav();

    @Override
    public Pose2d getUncorrectedPose() {
        return questNav.getPose();
    }

    @Override
    public Pose2d getCorrectedPose() {
        return getUncorrectedPose().transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        questNav.setPose(pose.transformBy(QuestNavConstants.ROBOT_TO_QUEST));

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
        inputs.timestamp = questNav.getDataTimestamp();
        inputs.timestampDelta = timestamp - inputs.timestamp;
        inputs.batteryLevel = questNav.getBatteryPercent();

        questNav.commandPeriodic();

    }
}
