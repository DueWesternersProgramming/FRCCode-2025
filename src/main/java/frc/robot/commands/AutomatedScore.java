package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.commands.drive.autoalign.AlignWithPose;
import frc.robot.RobotConstants.ScoringConstants;

public class AutomatedScore extends Command {
    Pose2d targetPose;
    double xOffset = .5;

    Supplier<Integer> reefSide;
    Supplier<Integer> position;
    Supplier<Integer> height;
    DriveSubsystem drivesubsystem;
    Field2d field = new Field2d();
    
    public AutomatedScore(Supplier<Integer> reefSide, Supplier<Integer> position, Supplier<Integer> height, DriveSubsystem drivesubsystem) {
        this.reefSide = reefSide;
        this.position = position;
        this.height = height;
        this.drivesubsystem = drivesubsystem;
        addRequirements(drivesubsystem);
    }

    @Override
    public void initialize() {
        targetPose = ScoringConstants.BlueAlliance.list.get(reefSide.get()-1);
        
        Translation2d translation = new Translation2d(xOffset, 0);

        if (position.get() == 0){
            xOffset = -xOffset;
        }
        if (position.get() == 1){
            xOffset = 0;
        }

        // Apply the translation to the target pose
        targetPose = targetPose.transformBy(new Transform2d(translation, targetPose.getRotation()));

        
        field.setRobotPose(targetPose);
        SmartDashboard.putData(field);
    }
    @Override
    public void execute() {
        System.out.println("Executing Automated Score");
        // Add commands here to be able to execute
    }
    @Override

    public void end(boolean interrupted) {
        // Add commands here to be able to execute
    }
    @Override
    public boolean isFinished() {
        return true;
    }

}
