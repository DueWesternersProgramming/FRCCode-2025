package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotState {

    public static boolean isManualControl = true;
    public static boolean canRotate = false;
    public static boolean xLocked = false;
    public static Pose2d robotPose = new Pose2d();
    public static boolean isAlgaeMode = false;

    public static void updatePose(Pose2d pose) {
        robotPose = pose;
    }

    public static Command setCanRotate(Boolean state) {
        return new InstantCommand(() -> canRotate = state);
    }

}