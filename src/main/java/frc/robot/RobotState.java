package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.AprilTagCamera;
import frc.robot.utils.TimestampedPose;

import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

    public static void visionPoseStatePeriodic(VisionSubsystem visionSubsystem, QuestNavSubsystem questSubsystem) {
        if (DriverStation.isEnabled()) {

            AprilTagCamera frontLeft = visionSubsystem.aprilTagCameras.get(0); // grab instance of camera
            // we use FL since it is the most reliable for the reef.
            if (frontLeft.inputs().hasTargets) {
                for (int i : VisionConstants.REEF_IDS) {
                    if (frontLeft.io().getBestTarget().fiducialId == i) {
                        // continue with logic here
                        if (frontLeft.io().getBestTarget().getArea() < .3) {
                            // don't use targets that are too far away.
                            break;
                        }
                        // Looking good, we can use FL camera pose results with confidence now
                        Logger.recordOutput("VisionSubsystem/ShouldUseFrontLeftPose", true);
                    }
                }
                Logger.recordOutput("VisionSubsystem/ShouldUseFrontLeftPose", false);
                return; // did not see any of the reef tags
            }
            Logger.recordOutput("VisionSubsystem/ShouldUseFrontLeftPose", false);

        }
        Logger.recordOutput("VisionSubsystem/ShouldUseFrontLeftPose", false);
    }
}
