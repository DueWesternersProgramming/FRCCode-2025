package frc.robot.subsystems.questnav;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants.QuestNavConstants;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNav questNav = new QuestNav();

    LoggedNetworkNumber questOffsetX = new LoggedNetworkNumber("/Tuning/QuestOffsetX", 0.0);
    LoggedNetworkNumber questOffsetY = new LoggedNetworkNumber("/Tuning/QuestOffsetY", 0.0);

    public static Transform2d ROBOT_TO_QUEST = new Transform2d(
            new Translation2d(0, 0),
            Rotation2d.kCCW_90deg);

    @Override
    public Pose2d getUncorrectedPose() {
        return questNav.getPose();
    }

    @Override
    public Pose2d getCorrectedPose() {
        return getUncorrectedPose().transformBy(ROBOT_TO_QUEST.inverse());
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        questNav.setPose(pose.transformBy(ROBOT_TO_QUEST));
    }

    @Override
    public boolean isConnected() {
        return questNav.isConnected();
    }

    @Override
    public void updateInputs(QuestIOInputs inputs) {

        ROBOT_TO_QUEST = new Transform2d(questOffsetX.get(), questOffsetY.get(), Rotation2d.kCCW_90deg);

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
