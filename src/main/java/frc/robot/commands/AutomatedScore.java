package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.commands.drive.autoalign.AlignWithPose;


public class AutomatedScore extends SequentialCommandGroup {
    Pose2d targetPose;

    public AutomatedScore(DriveSubsystem drivesubsystem) {
        targetPose = new Pose2d();
                
        addCommands(
                new PrintCommand("Score!"),
                AlignWithPose.pathToPoseCommand(targetPose, drivesubsystem)
                );
                
    }
}
