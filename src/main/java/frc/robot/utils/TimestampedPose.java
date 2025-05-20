package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public record TimestampedPose(Pose2d pose, double timestamp) {
}