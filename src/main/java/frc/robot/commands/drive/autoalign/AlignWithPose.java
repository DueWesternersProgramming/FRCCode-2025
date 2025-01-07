package frc.robot.commands.drive.autoalign;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.PathPlannerConstants;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemNeos;
import frc.robot.utils.CowboyUtils;

public class AlignWithPose {

    public AlignWithPose() {

    }

    public static Command pathToPoseCommand(Pose2d target) {      
        
        Command roughAlignmentCommand = AutoBuilder.pathfindToPose(
                target,
                PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS,
                0.0);

        return new SequentialCommandGroup(roughAlignmentCommand);
    }

    public static Command alignWithPoseCommand(Pose2d pose) {
        return pathToPoseCommand(CowboyUtils.testPose);
    }
}
