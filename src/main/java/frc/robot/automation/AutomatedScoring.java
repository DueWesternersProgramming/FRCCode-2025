package frc.robot.automation;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotConstants.ScoringConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.pathplanner.lib.util.FlippingUtil;

public class AutomatedScoring {
    static Pose2d targetPose;
    static double xOffset = .175;
    static double yOffset = .05;
    static Field2d field = new Field2d();

    public static Pose2d pathPlanToReef(Supplier<Integer> reefSide, Supplier<Integer> position) {
        // System.out.println("Reef Side: " + reefSide.get());
        targetPose = ScoringConstants.BlueAlliance.poses.get(reefSide.get() - 1);

        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);

            targetPose = new Pose2d(targetPose.getX(), targetPose.getY(),
                    new Rotation2d(Math.toRadians(targetPose.getRotation().getDegrees() + 90))); // Not sure what to do
                                                                                                 // about this
        }

        // Determine the correct x & y offset(s) based on the position
        double adjustedXOffset = xOffset;
        if (position.get() == 0) {
            adjustedXOffset = xOffset;
        } else if (position.get() == 1) {
            adjustedXOffset = 0;
        } else {
            adjustedXOffset = -xOffset;
        }

        // Create a translation for the offsets
        Translation2d translation = new Translation2d(yOffset, adjustedXOffset);

        // Apply the translation to the target pose
        targetPose = targetPose.transformBy(new Transform2d(translation, targetPose.getRotation()));

        return targetPose;

    }

    public static Command fullScore(Supplier<Integer> reefSide, Supplier<Integer> position,
            Supplier<Integer> height,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        Pose2d pose = pathPlanToReef(reefSide, position);

        return new ParallelCommandGroup(
                AlignWithPose.pathToPoseCommand(pose, drivesubsystem),
                new SequentialCommandGroup(elevatorSubsystem.goToScoreSetpoint(height.get())));
    }

    public static Command scoreNoPathing(int height, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {
        return new SequentialCommandGroup(elevatorSubsystem.goToScoreSetpoint(height),
                wristSubsystem.goToScoreSetpoint(height));
    }
}
