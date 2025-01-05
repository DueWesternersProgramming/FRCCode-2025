package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotState {

    public static boolean isManualControl = true;

    public static Pose2d robotPose = new Pose2d();

    public static void updatePose(Pose2d pose) {
        robotPose = pose;
    }

}