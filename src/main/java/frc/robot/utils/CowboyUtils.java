package frc.robot.utils;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CowboyUtils {
    public static AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            e.printStackTrace();
            aprilTagFieldLayout = null;
        }
    }

    public static Pose2d testPose = new Pose2d(1.4, 5.55, new Rotation2d(Math.toRadians(0)));

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Red) : (false);
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Blue) : (false);
    }
}