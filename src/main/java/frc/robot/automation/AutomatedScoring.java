package frc.robot.automation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotConstants.ScoringConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.pathplanner.lib.util.FlippingUtil;

public class AutomatedScoring {
    static Pose2d targetPose;
    static double xOffset = .2;
    static Field2d field = new Field2d();

    public static Pose2d pathPlanToReef(Supplier<Integer> reefSide, Supplier<Integer> position) {

        targetPose = ScoringConstants.BlueAlliance.poses.get(reefSide.get() - 1);

        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }

        // Determine the correct x offset based on the position
        double adjustedXOffset = xOffset;
        if (position.get() == 0) {
            adjustedXOffset = xOffset;
        } else if (position.get() == 1) {
            adjustedXOffset = 0;
        } else {
            adjustedXOffset = -xOffset;
        }

        // Create a translation for the x offset
        Translation2d translation = new Translation2d(0, adjustedXOffset);

        // Apply the translation to the target pose
        targetPose = targetPose.transformBy(new Transform2d(translation, targetPose.getRotation()));

        // field.setRobotPose(targetPose);
        // SmartDashboard.putData(field);
        return targetPose;

    }

    public static Command Score(Supplier<Integer> reefSide, Supplier<Integer> position,
            Supplier<Integer> height,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem) {

        Pose2d pose = pathPlanToReef(reefSide, position);

        return new ParallelCommandGroup(
                AlignWithPose.pathToPoseCommand(pose, drivesubsystem),
                ElevatorSubsystem.goToScoreSetpoint(height, elevatorSubsystem)
        // Add other commands here for automated scoring
        );
    }
}
