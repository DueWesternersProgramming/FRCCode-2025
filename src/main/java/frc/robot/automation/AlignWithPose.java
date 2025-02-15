package frc.robot.automation;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PathPlannerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignWithPose {

    public AlignWithPose() {

    }

    public static Command pathToPoseCommand(Pose2d target, DriveSubsystem driveSubsystem) {

        Command roughAlignmentCommand = AutoBuilder.pathfindToPose(
                target,
                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                0.0);

        return roughAlignmentCommand;
    }
}
