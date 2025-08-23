package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.TimestampedPose;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

    // public static boolean isManualControl = true;
    public static boolean canRotate = false;
    public static boolean xLocked = false;
    public static Pose2d robotPose = new Pose2d();
    @AutoLogOutput
    public static Boolean isQuestNavPoseReset = false;
    @AutoLogOutput
    public static Boolean manualQuestEnable = true;
    private static final Queue<TimestampedPose> questMeasurements = new LinkedBlockingQueue<>(20);
    private static final Queue<TimestampedPose> aprilTagCameraMeasurements = new LinkedBlockingQueue<>(20);

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

}