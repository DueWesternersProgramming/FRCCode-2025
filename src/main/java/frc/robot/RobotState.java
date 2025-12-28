package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.TimestampedPose;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
    public static Boolean isAntiTippingEnabled = true;
    public static Boolean isAutoAlignActive = false;
    public static boolean canRotate = false;
    public static boolean xLocked = false;
    public static Pose2d robotPose = new Pose2d();
    @AutoLogOutput
    public static Boolean isQuestNavPoseReset = false;
    @AutoLogOutput
    private static final Queue<TimestampedPose> questMeasurements = new LinkedBlockingQueue<>(20);
    private static final Queue<TimestampedPose> aprilTagCameraMeasurements = new LinkedBlockingQueue<>(20);

    public static enum AutoMode {
        PP_AUTO,
        DYNAMIC_AUTO
    }

    public static Queue<TimestampedPose> getQuestMeasurments() {
        return questMeasurements;
    }

    public static void offerQuestMeasurement(TimestampedPose observation) {
        questMeasurements.offer(observation);
    }

    public static Queue<TimestampedPose> getAprilTagCameraMeasurments() {
        return aprilTagCameraMeasurements;
    }

    public static void offerAprilTagCameraMeasurement(TimestampedPose observation) {
        aprilTagCameraMeasurements.offer(observation);
    }

    public static Command setCanRotate(Boolean state) {
        return new InstantCommand(() -> canRotate = state);
    }

    // public static void visionPoseStatePeriodic(VisionSubsystem visionSubsystem,
    // QuestNavSubsystem questSubsystem) {
    // // This method will keep track of the best pose source to use on the fly
    // (auto,
    // // autoalign, etc.), and
    // // set the state machine to the respective mode.

    // visionMode = VisionMode.QUEST_NAV_ONLY; // default to quest nav at the start
    // but change that below
    // if (questSubsystem.isConnected()) {

    // if (DriverStation.isEnabled()) {
    // AprilTagCamera frontLeft = visionSubsystem.aprilTagCameras.get(0); // grab
    // instance of camera
    // // we use FL since it is the most reliable results for the reef.
    // if (DriverStation.isAutonomous() || isAutoAlignActive) {

    // if (frontLeft.inputs().hasTargets) {
    // for (int i : VisionConstants.REEF_IDS) {
    // if (frontLeft.io().getBestTarget().fiducialId == i) {
    // double tagDistance =
    // frontLeft.io().getBestTarget().bestCameraToTarget.getTranslation()
    // .getNorm();
    // Logger.recordOutput("VisionSubsystem/TagDistance", tagDistance);
    // // distance is less than 2.75 meters (about 9 ft)
    // if (tagDistance < 2.75) {
    // visionMode = VisionMode.APRIL_TAG_ONLY;
    // break;
    // // If in auto or autoalign is active, and we see a reef tag within reasonable
    // // distance, use it only.
    // }
    // }
    // }
    // }
    // }
    // } else {
    // visionMode = VisionMode.APRIL_TAG_ONLY; // april tag source only when
    // disabled
    // }

    // }

    // else {
    // visionMode = VisionMode.APRIL_TAG_ONLY; // april tag source only when
    // disabled
    // }

    // Logger.recordOutput("VisionSubsystem/QuestConnected",
    // questSubsystem.isConnected());

    // Logger.recordOutput("VisionSubsystem/VisionSourceMode",
    // visionMode.toString()); // log vision mode for debugging

    // Logger.recordOutput("RobotState/AutoAlignActive", isAutoAlignActive);
    // }
}
