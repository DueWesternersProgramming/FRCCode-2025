package frc.robot.automation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotConstants.ScoringConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AutomatedScoring {
    static Pose2d targetPose;
    static double xOffset = .2;
    static Field2d field = new Field2d();

    public static Pose2d pathPlanToReef(int reefSide, int position) {
        targetPose = CowboyUtils.isBlueAlliance() ? ScoringConstants.BlueAlliance.poses.get(reefSide - 1)
                : ScoringConstants.RedAlliance.poses.get(reefSide - 1);

        // Determine the correct x offset based on the position
        double adjustedXOffset = xOffset;
        if (position == 0) {
            adjustedXOffset = xOffset;
        } else if (position == 1) {
            adjustedXOffset = 0;
        } else {
            adjustedXOffset = -xOffset;
        }

        // Create a translation for the x offset
        Translation2d translation = new Translation2d(0, adjustedXOffset);

        // Apply the translation to the target pose
        targetPose = targetPose.transformBy(new Transform2d(translation, targetPose.getRotation()));

        System.out.println(targetPose);
        field.setRobotPose(targetPose);
        SmartDashboard.putData(field);
        return targetPose;

    }

    public static Command Score(Supplier<Integer> reefSide, Supplier<Integer> position,
            Supplier<Integer> height,
            DriveSubsystem drivesubsystem) {

        Pose2d pose = pathPlanToReef(reefSide.get(), position.get());

        return new SequentialCommandGroup(
                AlignWithPose.pathToPoseCommand(pose, drivesubsystem)
        // Add other commands here for automated scoring
        );
    }
}
